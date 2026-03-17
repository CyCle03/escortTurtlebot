# Escort Turtlebot Architecture

This document describes the core architecture of the `escort_turtlebot_pkg` and the hybrid leader-follower approach used to orchestrate multiple TurtleBot3 robots.

## 1. System Overview

The primary goal of this package is to orchestrate a "Leader" and a "Follower" robot in a ROS 2 simulation (and adaptable to real hardware). The environment leverages Nav2 and SLAM Toolbox to enable autonomous navigation and collaborative movement.

### Hybrid Follower Approach
The tracking system uses a hybrid method combining several ideas:
1.  **Leader Rear-Targeting**: An idea from the `team_project` where the follower tracks a virtual target point projected behind the leader.
2.  **Nav2 `FollowPath` Execution**: Utilizing Nav2 (`escort_follower`) on the follower robot to navigate to the continuously updated target positions.
3.  **Local Obstacle Avoidance**: The follower (`TB3_2`) does not run a heavy global SLAM map. Instead, it relies on its local LiDAR data (`/TB3_2/scan`) to avoid dynamic obstacles using Nav2's local costmap.

## 2. Core Nodes

### `follower_detector_node`
This is arguably the most critical node for multi-robot synchronization. It eliminates the need to precisely and perfectly place the follower robot relative to the leader at startup.

**How it works (ICP LiDAR Matching):**
-   Subscribes to the LiDAR scan data of both robots (`/TB3_1/scan` and `/TB3_2/scan`).
-   Uses the **ICP (Iterative Closest Point)** algorithm to continuously match these two point clouds.
-   By finding how the two point clouds overlap, it calculates the exact relative position and orientation between the leader and follower.
-  - **TF Synchronization**: It dynamically computes and broadcasts the transform (`TF`) between `TB3_1/odom` and `TB3_2/odom`. This effectively aligns the follower's local odometry into the leader's global map frame in real-time.

**Stability & Robustness (SLAM Safeguards):**
To prevent the map from "wobbling" or jumping due to bad sensor data:
- **Motion Gating**: If the follower's odometry shows it's not moving significantly (`odom_motion_threshold`), ICP updates are skipped to avoid introducing noise while stationary.
- **Correction Limits**: ICP corrections exceeding a certain distance (`max_correction_dist`) are rejected as false positives.
- **Alpha Blending**: New ICP results are blended with the previous TF bridge state (`blend_alpha`) to ensure smooth transitions.
- **Stale Data Rejection**: If the leader's scan is not received within a timeout, the system stops updating and maintains the last known valid state.

### Recovery Behaviors
The system handles edge cases where the leader might be temporarily lost:
- **Wait at Last Known Position**: If ICP fails to find a high-fitness match or scans are lost, the `follower_detector_node` continues to broadcast the last known valid `TB3_1/odom` -> `TB3_2/odom` transform. This ensures the follower stays put relative to the leader's last known position rather than drifting away or causing a TF disconnect.

## 3. TF Tree Structure

In a multi-robot setup, managing the TF (Transform) tree is vital. Because both robots use `Nav2`, their coordinate frames must be cleanly separated using namespaces but logically connected.

### Namespace Isolation
-   **Leader**: `TB3_1` -> Frames: `TB3_1/odom`, `TB3_1/base_footprint`, `TB3_1/base_scan`
-   **Follower**: `TB3_2` -> Frames: `TB3_2/odom`, `TB3_2/base_footprint`, `TB3_2/base_scan`

### Map to Odometry Connection
`slam_toolbox` (running on the leader) provides the `map` -> `TB3_1/odom` transform.

### The Odom Bridge
The `follower_detector_node` acts as the vital bridge linking the two robots:
`TB3_1/odom` -> `TB3_2/odom`

This complete chain allows the follower robot (`TB3_2/base_footprint`) to have a mathematically defined position within the global `map` created by the leader, enabling it to receive global navigation goals relative to the leader's position.

---

## 한국어 설명 (Korean Explanation)

# Escort Turtlebot 아키텍처

이 문서는 `escort_turtlebot_pkg`의 핵심 구조와 여러 대의 터틀봇 3(TurtleBot3)를 제어하기 위해 사용된 하이브리드 리더-팔로워 방식에 대해 설명합니다.

## 1. 시스템 개요

이 패키지의 주요 목표는 ROS 2 시뮬레이션(또는 실제 하드웨어) 내에서 "리더(Leader)"와 "팔로워(Follower)" 로봇을 제어하는 것입니다. 이 환경은 Nav2와 SLAM Toolbox를 바탕으로 자율 주행 및 협업 주행을 구현합니다.

### 하이브리드 추종(Follower) 방식
추적 시스템은 다음과 같은 아이디어들을 결합한 하이브리드 방식을 사용합니다:
1.  **리더 후방 목표점 방식**: `team_project`의 아이디어로, 팔로워가 리더의 후방에 투사된 가상의 타겟을 추종하도록 합니다.
2.  **Nav2 `FollowPath` 실행**: 팔로워 로봇(`escort_follower`)에서 Nav2를 사용하여 지속적으로 업데이트되는 타겟 위치로 주행합니다.
3.  **로컬 장애물 회피**: 팔로워(`TB3_2`)는 무거운 글로벌 SLAM 맵을 구동하지 않습니다. 대신 자체 LiDAR 데이터(`/TB3_2/scan`)와 Nav2의 로컬 코스트맵(local costmap)을 활용하여 동적 장애물을 회피합니다.

## 2. 핵심 노드 (Core Nodes)

### `follower_detector_node`
이 노드는 다중 로봇 동기화에 있어 가장 핵심적인 역할을 합니다. 덕분에 시작 시 리더 대비 팔로워 로봇의 물리적 위치를 정확하게 맞출 필요가 사라집니다.

**동작 원리 (ICP LiDAR 매칭):**
- 두 로봇의 LiDAR 스캔 데이터(`/TB3_1/scan`, `/TB3_2/scan`)를 구독합니다.
- **ICP (Iterative Closest Point)** 알고리즘을 사용해 두 점군(Point Cloud) 데이터를 지속적으로 비교 및 대조합니다.
- 두 점군 데이터가 겹쳐지는 형상을 찾아내어, 리더 로봇 대비 팔로워 로봇의 정확한 상대 위치(x, y)와 방향(yaw)을 계산해 냅니다.
- **TF 동기화**: `TB3_1/odom`과 `TB3_2/odom` 사이의 변환 행렬(`TF`)을 실시간으로 계산하여 방송(Broadcast)합니다. 이를 통해 팔로워의 로컬 오도메트리 좌표계를 리더의 글로벌 맵 프레임에 맞춰 실시간으로 정렬시키며, 결과적으로 팔로워가 맵 서비스 없이도 리더의 맵 상에서 자신의 위치를 파악할 수 있게 합니다.

**안정성 및 강인성 (SLAM 보호 장치):**
불완전한 센서 데이터로 인해 맵이 흔들리거나 로봇 위치가 튀는 현상을 방지하기 위해 다음과 같은 보호 로직이 적용되었습니다:
- **이동 기반 제어 (Motion Gating)**: 팔로워 로봇이 정지해 있거나 이동량이 매우 적을 경우(`odom_motion_threshold` 미만), 불필요한 노이즈가 TF 트리에 반영되어 맵을 오염시키는 것을 막기 위해 ICP 업데이트를 건너뜁니다.
- **보정 범위 제한 (Correction Limits)**: 한 번의 보정으로 변화할 수 있는 최대 거리(`max_correction_dist`)를 제한하여, 갑작스러운 스캔 오류로 인해 로봇 위치가 점프하는 현상을 원천 차단합니다.
- **알파 블렌딩 (Alpha Blending)**: 새로운 ICP 계산 결과를 이전 TF 상태와 일정 비율(`blend_alpha`)로 부드럽게 혼합하여, 좌표계 이동이 급격하지 않고 매끄럽게 이루어지도록 합니다.
- **데이터 유효성 타임아웃**: 리더 로봇의 스캔 데이터 소실 시, 잘못된 정합을 시도하는 대신 마지막으로 확인된 유효 상태를 유지합니다.

### 복구 행동 (Recovery Behaviors)
주행 중 리더 로봇을 일시적으로 놓치거나 센서 데이터가 불안정해지는 예외 상황에 대응합니다:
- **마지막 위치 대기 (Wait at Last Known Position)**: ICP 매칭 품질(fitness)이 낮거나 리더 로봇의 데이터가 끊겼을 때, `follower_detector_node`는 마지막으로 성공했던 `TB3_1/odom` -> `TB3_2/odom` 변환 값을 계속해서 유지합니다. 이를 통해 팔로워 로봇은 맵 상에서 미아가 되지 않고 리더의 마지막 확인 위치를 기준으로 안정적으로 대기하며 다시 정합이 성공하기를 기다립니다.

## 3. TF 트리 (TF Tree Structure)

다중 로봇 설정에서는 TF 트리 관리가 매우 중요합니다. 두 로봇 모두 `Nav2`를 사용하기 때문에 네임스페이스를 통해 좌표계를 명확히 분리하면서도 논리적으로 연결해야 합니다.

### 네임스페이스 격리 (Namespace Isolation)
-   **리더(Leader)**: `TB3_1` -> 사용 프레임: `TB3_1/odom`, `TB3_1/base_footprint`, `TB3_1/base_scan`
-   **팔로워(Follower)**: `TB3_2` -> 사용 프레임: `TB3_2/odom`, `TB3_2/base_footprint`, `TB3_2/base_scan`

### Map과 Odometry 연결
리더 로봇에서 동작하는 `slam_toolbox`가 `map` -> `TB3_1/odom` 변환 트리를 제공합니다.

### Odom 브릿지 (The Odom Bridge)
`follower_detector_node` 노드가 두 로봇을 연결하는 결정적인 역할을 합니다:
`TB3_1/odom` -> `TB3_2/odom`

이러한 전체 연결 고리를 통해 팔로워 로봇(`TB3_2/base_footprint`)은 리더가 생성한 글로벌 `map` 상에서 수학적으로 명확한 자신의 위치를 가질 수 있으며, 리더의 위치를 기준으로 된 내비게이션 목표를 수행할 수 있게 됩니다.
