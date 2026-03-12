# System Architecture & TF Tree

This document provides an in-depth look at the architecture of the **escort_turtlebot_pkg** project and how multiple robots coordinate in real-time.

## Hybrid Follower Architecture

Traditional multi-robot following systems usually rely on either:
1. Strict Odometry tracking (prone to cumulative drift errors).
2. Pure Vision/LiDAR following without a global map (cannot use global path planning).

The **escort_turtlebot_pkg** employs a **Hybrid Architecture**:
- **Leader (`{leader_name}`)** generates the global map and navigates using full SLAM (`slam_toolbox`) or predefined maps.
- **Follower (`{follower_name}`)** does *not* run a heavy SLAM node. Instead, it relies on the leader's map and uses its own LiDAR purely for the **Local Costmap** (Dynamic Obstacle Avoidance).
- The `follower_detector_node` acts as the bridge. It matches the shape of the physical surroundings seen by both robots' LiDARs (using **ICP Scan Matching**) to synchronize their Odometry (`odom`) frames.

### Automatic Recovery Behavior
If the follower loses sight of the leader or the TF drops (due to occlusion or network lag), the follower immediately enters a standard **Recovery Mode**:
- The active tracking goal is canceled.
- The follower sends a new static goal to the **last known position** of the leader and waits there.
- Once the TF/network is restored and the leader is detected again, the follower smoothly resumes tracking from that spot.

### Map Stability Protection (ICP Guard)

When the follower gets stuck behind an obstacle its wheels slip, causing odometry drift. If ICP runs in this state, the two LiDAR scans see different environments and produce a falsely large TF correction — which can cause the SLAM map to explode radially.

To prevent this, the `follower_detector_node` applies three layered guards:

| Guard | Trigger | Action |
|---|---|---|
| **Stationary detection** | Follower odom change < `odom_motion_threshold` (0.02 m) | Skip ICP entirely |
| **Correction size limit** | ICP result deviates > `max_correction_dist` (0.3 m) or `max_correction_angle` (0.5 rad) from current TF | Reject ICP result |
| **Leader scan timeout** | No `/{leader_name}/scan` for > `scan_timeout_sec` (1.0 s) | Preserve last valid bridge TF |

Additionally, `slam_toolbox` is configured conservatively (`loop_match_maximum_variance_coarse: 1.5`) to reject ambiguous loop closures, which are the direct cause of radial map explosions.

## The TF Tree Structure

In ROS 2 Navigation, Transforms (TF) are critical. Our multi-robot system unifies two separate robots under a single `map` frame using dynamic ICP correction.

```mermaid
graph TD;
    map-->{leader_name}/odom;
    
    %% Leader Branch
    {leader_name}/odom-->{leader_name}/base_footprint;
    {leader_name}/base_footprint-->{leader_name}/base_link;
    {leader_name}/base_link-->{leader_name}/base_scan;
    
    %% Follower Branch bridged by follower_detector_node
    {leader_name}/odom-. "follower_detector_node (ICP Correction)" .->{follower_name}/odom;
    
    {follower_name}/odom-->{follower_name}/base_footprint;
    {follower_name}/base_footprint-->{follower_name}/base_link;
    {follower_name}/base_link-->{follower_name}/base_scan;
```

### Key Components

1. **`map`**: The global frame created by the Leader's SLAM module.
2. **`{leader_name}/odom`**: The dead-reckoning starting point of the Leader.
3. **`{leader_name}/odom` -> `{follower_name}/odom`**: This transform is dynamically published at 10Hz by the `follower_detector_node`. It calculates the offset between the two robots by aligning `{leader_name}/scan` with `{follower_name}/scan`. This automatically corrects any drift in the follower's wheels.

## Action Flow for Follower Nodes

1. **Leader Control**: Operator sends `cmd_vel` to `/{leader_name}/cmd_vel`. 
2. **Scan Matching**: `follower_detector_node` receives both robots' scans, runs ICP, and updates the `{follower_name}/odom` TF.
3. **Target Pose Generation**: `Follower` class (in `escort_follower` package) reads the real-time TF. It looks at the leader's heading, computes a target spot exactly `0.5m` behind the leader, and sends a `FollowPath` Action to `/{follower_name}/follow_path`.
4. **Local Planning**: The Nav2 stack on the follower computes the motor velocities to reach that spot while dodging local obstacles (people, boxes) that appear on its own LiDAR.

---

## 한국어 안내 (System Architecture & TF Tree)

이 문서는 **escort_turtlebot_pkg** 프로젝트의 아키텍처와 다중 로봇 실시간 제어 원리에 대해 심도 있게 다룹니다.

### 하이브리드(Hybrid) 팔로워 아키텍처

기존의 다중 로봇 추종 시스템은 대개 다음 두 가지 중 하나를 사용합니다:
1. 단순 오도메트리(Odometry) 추적: 누적 오차(Drift error)에 매우 취약함.
2. 글로벌 맵이 없는 순수 비전/LiDAR 추종: 글로벌 경로 탐색(A* 등)이 불가능함.

본 프로젝트는 두 방식의 장점을 합친 **하이브리드 아키텍처**를 사용합니다:
- **리더 (`{leader_name}`)**: 전체 맵(SLAM)을 생성하거나 기존 맵 위에서 글로벌 내비게이션(`slam_toolbox`)을 수행합니다.
- **팔로워 (`{follower_name}`)**: 무거운 SLAM 노드를 띄우지 않습니다. 대신 리더가 만든 맵을 공유하며, 자신의 LiDAR는 오직 **로컬 코스트맵 (동적 장애물 회피)** 용도로만 사용합니다.
- **`follower_detector_node`**: 두 로봇의 가교 역할을 합니다. 양쪽 로봇의 LiDAR 스캔이 만들어내는 물리적 윤곽선을 겹쳐보는 방식(**ICP Scan Matching**)으로 두 로봇의 주행 기록(`odom`)을 오차 없이 실시간 동기화합니다.

#### 지능형 예외 상황 복구 행동 (Recovery Behavior)
장애물에 가려지거나 네트워크 지연으로 인해 팔로워가 리더의 위치 정보(TF)를 놓칠 경우, 팔로워는 즉시 다음과 같은 **복구 모드**에 돌입합니다:
- 현재 진행 중이던 내비게이션 추종 목표를 취소합니다.
- 리더가 마지막으로 목격되었던 가장 확실한 좌표로 새로운 목표점을 설정하여 이동하고, 그 자리에 정지하여 대기합니다.
- 네트워크나 시야가 회복되어 리더 TF가 다시 잡히면, 그 순간부터 즉각적으로 추종을 재개합니다.

#### 맵 안정성 보호 (ICP Guard)

팔로워가 장애물에 막히면 바퀴가 미끄러지며 오도메트리가 드리프트합니다. 이 상태에서 ICP가 실행되면 두 LiDAR가 서로 다른 환경을 보게 되어 크게 잘못된 TF 보정값이 생성되고, SLAM 맵이 방사형으로 폭발적으로 오염될 수 있습니다.

이를 방지하기 위해 `follower_detector_node`는 세 단계의 보호 로직을 적용합니다:

| 보호 단계 | 트리거 조건 | 동작 |
|---|---|---|
| **정지 상태 감지** | 팔로워 odom 변화량 < `odom_motion_threshold` (0.02 m) | ICP 전체 건너뜀 |
| **보정 크기 제한** | ICP 결과가 현재 TF 대비 > `max_correction_dist` (0.3 m) 또는 `max_correction_angle` (0.5 rad) | ICP 결과 reject |
| **리더 스캔 타임아웃** | `/{leader_name}/scan`이 `scan_timeout_sec` (1.0 s) 이상 없음 | 마지막 유효 bridge TF 보존 |

또한 `slam_toolbox`를 보수적으로 설정하여(`loop_match_maximum_variance_coarse: 1.5`) 잘못된 루프 클로저(방사형 맵 폭발의 직접 원인)를 원천 차단합니다.

### TF(Transform) 트리 구조

ROS 2 내비게이션에서 가장 중요한 것은 TF(좌표계 변환)입니다. 이 시스템은 ICP 보정을 통해 서로 독립된 두 대의 로봇을 하나의 `map` 프레임 아래로 통합합니다.

```mermaid
graph TD;
    map-->{leader_name}/odom;
    
    %% Leader Branch
    {leader_name}/odom-->{leader_name}/base_footprint;
    {leader_name}/base_footprint-->{leader_name}/base_link;
    {leader_name}/base_link-->{leader_name}/base_scan;
    
    %% Follower Branch bridged by follower_detector_node
    {leader_name}/odom-. "follower_detector_node (ICP 보정)" .->{follower_name}/odom;
    
    {follower_name}/odom-->{follower_name}/base_footprint;
    {follower_name}/base_footprint-->{follower_name}/base_link;
    {follower_name}/base_link-->{follower_name}/base_scan;
```

#### 핵심 구성 요소

1. **`map`**: 리더 로봇의 SLAM이 만들어낸 전역 좌표계.
2. **`{leader_name}/odom`**: 리더 로봇의 바퀴 회전량(추측 항법) 기준점.
3. **`{leader_name}/odom` -> `{follower_name}/odom`**: `follower_detector_node`가 10Hz로 쏴주는 동적 TF입니다. `{leader_name}/scan`과 `{follower_name}/scan`을 정합하여 두 로봇 간의 오프셋을 계산하며, 이 덕분에 팔로워 로봇 바퀴의 미끄러짐(Drift) 오차가 실시간으로 깔끔하게 보정됩니다.

### 팔로워 노드의 동작 흐름 (Action Flow)

1. **리더 조종**: 사용자가 `/{leader_name}/cmd_vel`로 명령을 내립니다.
2. **스캔 매칭**: `follower_detector_node`가 양쪽 LiDAR를 스캔하여 오차를 수정한 `{follower_name}/odom` TF를 업데이트합니다.
3. **목표점 생성**: (`escort_follower` 패키지의) `Follower` 클래스가 실시간 TF를 읽고, 리더의 진행 방향을 계산하여 정확히 0.5m 뒤의 위치를 찍은 다음 `/{follower_name}/follow_path` 액션을 호출합니다.
4. **로컬 플래닝**: 팔로워 로봇의 Nav2 스택이 자신의 LiDAR에 찍히는 주변 장애물(사람, 박스)을 피하면서 목표점까지 가는 최적의 모터 속도를 계산하여 주행합니다.
