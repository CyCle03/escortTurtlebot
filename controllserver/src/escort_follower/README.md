# escort_follower

C++ package for multi-TurtleBot follower control using TF-relative target generation and Nav2 `FollowPath`.

## What It Does
- Runs follower nodes for `TB3_2 ... TB3_n`.
- Computes leader pose in follower frame from TF.
- Builds short 2-point path and sends `FollowPath` goals.

## Build
```bash
cd ~/escort_ws/controllserver
colcon build --packages-select escort_follower
source install/setup.bash
```

## Run (Example)
```bash
ros2 launch escort_turtlebot_pkg escort_follower.launch.py \
  number_of_follower:=1 use_lidar_bridge:=true use_sim_time:=false
```

## Main Parameters (`follower` node)
- `follow_distance` (double, default: `0.5`)
  Desired distance to keep from leader.
- `publish_odom_bridge` (bool, default: `true`)
  Enable internal TF bridge publishing (`leader/odom -> follower/odom`).
- `goal_update_distance_threshold` (double, default: `0.03`)
  Minimum target change before sending a new `FollowPath` goal.
- `goal_update_min_period_sec` (double, default: `0.3`)
  Minimum interval between consecutive goals.
- `use_sim_time` (bool, default: `false`)
  Use simulation clock.

## Notes
- TF lookup failures are skipped safely (no stale goal resend on TF miss).
- Goal updates are rate-limited and change-filtered to reduce Nav2 action spam.
- Distance sensing/calculation reference:
  - `docs/distance_measurement_reference.md`

## 한국어 안내

이 패키지는 TF 기반 상대 위치를 이용해 follower 경로를 생성하고 Nav2 `FollowPath`로 전달합니다.

### 기능
- `TB3_2 ... TB3_n` follower 노드 실행
- follower 기준 leader 상대 위치 계산
- 2개 포인트 경로 생성 후 `FollowPath` goal 전송

### 빌드
```bash
cd ~/escort_ws/controllserver
colcon build --packages-select escort_follower
source install/setup.bash
```

### 실행 예시
```bash
ros2 launch escort_turtlebot_pkg escort_follower.launch.py number_of_follower:=1 use_lidar_bridge:=true use_sim_time:=false
```

### 주요 파라미터 (`follower` 노드)
- `follow_distance` (기본값 `0.5`): 리더와 유지할 목표 거리
- `publish_odom_bridge` (기본값 `true`): 내부 TF 브리지(`leader/odom -> follower/odom`) 사용 여부
- `goal_update_distance_threshold` (기본값 `0.03`): 새 goal 전송을 위한 최소 목표 변화량
- `goal_update_min_period_sec` (기본값 `0.3`): goal 전송 최소 주기(초)
- `use_sim_time` (기본값 `false`): 시뮬레이션 시간 사용 여부

### 참고
- TF 조회 실패 시 goal 전송을 건너뛰도록 처리되어 있습니다.
- goal 전송은 변화량/주기 필터가 적용되어 액션 과전송을 줄입니다.
- 거리 인식/계산 참고 문서:
  - `docs/distance_measurement_reference.md`
