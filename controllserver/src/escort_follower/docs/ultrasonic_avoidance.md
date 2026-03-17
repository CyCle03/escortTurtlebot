# Ultrasonic Emergency Avoidance

This document describes the design and logic of the ultrasonic-based emergency obstacle avoidance integrated into the `follower` node.

## Overview
To improve the safety of the follower robot (`TB3_2`), an ultrasonic sensor is used to detect obstacles at extremely close ranges that might be missed by the LiDAR or not yet reflected in the Nav2 costmap.

## Logic Flow

1. **Subscription**: The node subscribes to the `/ultrasonic_distance` topic (unit: cm).
2. **Threshold**: An emergency threshold is set at **10.0 cm**.
3. **Emergency Trigger**:
   - If `distance <= 10.0 cm`:
     - **Cancel Nav2 Goal**: The active `FollowPath` goal is immediately canceled to prevent the Nav2 controller from fighting the emergency command.
     - **Reverse Command**: A steady reverse velocity of **-0.07 m/s** is published directly to the robot's velocity topic.
     - **Path Suppression**: New path generation and goal sending in `send_path()` are suppressed while in this state.
     - **Strict Precedence**: Emergency logic acts as an absolute gate (`if/elif` structure). Any target tracking updates are ignored to prevent tracking commands from overriding the reverse sequence.
4. **Recovery**:
   - If `distance > 10.0 cm`:
     - The emergency flag is cleared.
     - A stop command (`0.0 m/s`) is published once.
     - Normal tracking logic resumes on the next timer cycle.

## Topic Mapping
- **Input**: `/ultrasonic_distance` (`std_msgs/Float32`)
- **Output**: `/{follower_name}/cmd_vel` (`geometry_msgs/Twist`)

## Implementation Details
- **Flag**: `is_emergency_` (bool)
- **Callback**: `sonar_callback()`
- **Safety**: Direct velocity publishing ensures low-latency response regardless of the Nav2 pipeline state.

## 한국어 안내

이 문서는 `follower` 노드에 통합된 초음파 기반 긴급 장애물 회피 로직의 설계 및 동작 방식을 설명합니다.

### 개요
팔로워 로봇(`TB3_2`)의 안전성을 높이기 위해, LiDAR가 놓치거나 Nav2 코스트맵에 즉시 반영되지 않는 초근거리 장애물을 감지하고자 초음파 센서를 사용합니다.

### 동작 로직

1. **구독**: `/ultrasonic_distance` 토픽(단위: cm)을 구독합니다.
2. **임계값**: 긴급 회피 임계값은 **10.0 cm**로 설정되어 있습니다.
3. **긴급 모드 트리거**:
   - `거리 <= 10.0 cm`인 경우:
     - **Nav2 목표 취소**: 실행 중인 `FollowPath` 액션 목표를 즉시 취소하여 Nav2 제어기와 긴급 명령이 충돌하는 것을 방지합니다.
     - **후진 명령**: **-0.07 m/s**의 일정한 후진 속도를 로봇의 속도 토픽으로 직접 발행합니다.
     - **경로 생성 억제**: 이 상태가 유지되는 동안 `send_path()`를 통한 새로운 경로 생성 및 목표 전송이 중단됩니다.
     - **우선순위 강제**: 긴급 회피 로직은 제어 루프 내에서 최상단 우선 조건으로 처리되며(`if/elif` 제어), 이 상태에서는 다른 추종 관련 연산이 무시되어 뒤집어쓰이는 현상을 방지합니다.
4. **복구**:
   - `거리 > 10.0 cm`인 경우:
     - 긴급 플래그를 해제합니다.
     - 정지 명령(`0.0 m/s`)을 1회 발행합니다.
     - 다음 타이머 주기부터 정상 추적 로직을 재개합니다.

### 토픽 매핑
- **입력**: `/ultrasonic_distance` (`std_msgs/Float32`)
- **출력**: `/{follower_name}/cmd_vel` (`geometry_msgs/Twist`)

### 구현 세부 사항
- **플래그**: `is_emergency_` (bool)
- **콜백**: `sonar_callback()`
- **안전성**: 속도 명령을 직접 발행함으로써 Nav2 파이프라인 상태와 관계없이 저지연 응답을 보장합니다.
