# escort_follower

C++ package for multi-robot follower control using TF-relative target generation and Nav2 `FollowPath`.

## What It Does
- Runs follower nodes for `TB3_2 ... TB3_n`.
- Computes leader pose in the follower's frame from TF.
- Builds a short 2-point path and sends `FollowPath` goals.
- Uses a hybrid target rule: the target point is generated behind the leader's heading.
- The core logic is compiled into a reusable `follower_lib` library.
- **NEW: Ultrasonic Emergency Avoidance**: Subscribes to `/ultrasonic_distance` and triggers emergency reverse if an obstacle is within 10cm.

## Build
```bash
cd ~/escort_ws/controllserver
colcon build --packages-select escort_follower
source install/setup.bash
```

## Run

### Via Launch File (Recommended)
```bash
ros2 launch escort_turtlebot_pkg escort_follower.launch.py \
  leader_name:=TB3_1 \
  follower_name:=TB3_2 \
  follow_distance:=0.5 \
  initial_step_distance:=0.0 \
  use_sim_time:=false
```

### Standalone (For Debugging)
The follower node can be run directly. It requires the follower and leader names as command-line arguments.
```bash
ros2 run escort_follower follower <follower_name> <leader_name> --ros-args -p use_sim_time:=false
```

## Main Parameters (`follower` node)
- `follow_distance` (double, default: `0.5`)
  Rear offset from leader heading used to generate the follow target.
- `initial_step_distance` (double, default: `0.0`)
  One-time initial forward step distance toward the generated target.
- `goal_update_distance_threshold` (double, default: `0.03`)
  Minimum target change before sending a new `FollowPath` goal.
- `goal_update_min_period_sec` (double, default: `0.3`)
  Minimum interval between consecutive goals.
- `use_sim_time` (bool, default: `false`)
  Use simulation clock.
- `tf_timeout_sec` (double, default: `2.0`)
  Seconds without a valid leader TF before entering Recovery Mode.
- `recovery_resend_period_sec` (double, default: `3.0`)
  While in Recovery Mode, re-send the "go to last known position" goal every N seconds.
  Prevents the follower from stopping if Nav2 silently drops the goal.

## Notes
- TF lookup failures are skipped safely; an error is logged via `WARN` (throttled) for debugging.
- Goal updates are rate-limited and change-filtered to reduce Nav2 action spam.
- **Recovery Mode:** when the leader's TF is lost for `tf_timeout_sec`, the follower navigates to the leader's last known position and **re-sends the goal every `recovery_resend_period_sec`** seconds until the leader is found again.
- **Emergency Mode**: When ultrasonic distance <= 10.0cm, active Nav2 goals are canceled and the robot publishes a reverse command (`-0.07m/s`) directly to `/cmd_vel`.
- Distance sensing/calculation reference:
  - [`docs/distance_measurement_reference.md`](docs/distance_measurement_reference.md)
  - [`docs/ultrasonic_avoidance.md`](docs/ultrasonic_avoidance.md) (Design of emergency reverse)

## 한국어 안내

이 패키지는 TF 기반 상대 위치를 이용해 follower 경로를 생성하고 Nav2 `FollowPath`로 전달합니다.

### 기능
- `TB3_2 ... TB3_n` follower 노드 실행
- follower 기준 leader 상대 위치 계산
- 2개 포인트 경로 생성 후 `FollowPath` goal 전송
- 하이브리드 타겟 규칙 사용: 타겟 포인트는 리더의 방향 뒤에 생성됩니다.
- 핵심 로직은 재사용 가능한 `follower_lib` 라이브러리로 컴파일됩니다.
- **추가됨: 초음파 긴급 회피**: `/ultrasonic_distance` 토픽을 통해 10cm 이내 장애물 감지 시 즉시 긴급 후진을 수행합니다.

### 빌드
```bash
cd ~/escort_ws/controllserver
colcon build --packages-select escort_follower
source install/setup.bash
```

### 실행

#### 런치 파일 사용 (권장)
```bash
ros2 launch escort_turtlebot_pkg escort_follower.launch.py \
  leader_name:=TB3_1 \
  follower_name:=TB3_2 \
  follow_distance:=0.5 \
  initial_step_distance:=0.0 \
  use_sim_time:=false
```

#### 단독 실행 (디버깅용)
팔로워 노드를 직접 실행할 수 있습니다. 팔로워와 리더 이름을 커맨드 라인 인자로 받습니다.
```bash
ros2 run escort_follower follower <팔로워_이름> <리더_이름> --ros-args -p use_sim_time:=false
```

### 주요 파라미터 (`follower` 노드)
- `follow_distance` (기본값 `0.5`): 리더 진행 방향 기준 뒤쪽 목표점 오프셋
- `initial_step_distance` (기본값 `0.0`): follower의 1회 초기 전진 거리
- `goal_update_distance_threshold` (기본값 `0.03`): 새 goal 전송을 위한 최소 목표 변화량
- `goal_update_min_period_sec` (기본값 `0.3`): goal 전송 최소 주기(초)
- `use_sim_time` (기본값 `false`): 시뮬레이션 시간 사용 여부
- `tf_timeout_sec` (기본값 `2.0`): 리더의 위치(TF)를 잃어버렸을 때 복구 모드 진입까지 대기 시간(초)
- `recovery_resend_period_sec` (기본값 `3.0`): 복구 모드 중 "마지막 위치로 이동" 목표를 재전송하는 주기(초)
  Nav2가 목표를 조용히 삭제해도 팔로워가 멈추지 않도록 보장합니다.

### 참고
- TF 조회 실패 시 `WARN` 로그(Throttled)로 기록되며 goal 전송은 건너뜁니다.
- goal 전송은 변화량/주기 필터가 적용되어 액션 과전송을 줄입니다.
- **복구 모드:** 리더 TF가 `tf_timeout_sec` 이상 끊기면 마지막 알려진 위치로 이동하고, `recovery_resend_period_sec` 주기마다 목표를 재전송하여 리더가 돌아올 때까지 대기합니다.
- **긴급 모드**: 초음파 거리가 10cm 이하인 경우, 실행 중인 Nav2 목표를 취소하고 `/cmd_vel`로 직접 후진 명령(-0.07m/s)을 전송합니다.
- 거리 인식/계산 참고 문서:
  - [`docs/distance_measurement_reference.md`](docs/distance_measurement_reference.md)
  - [`docs/ultrasonic_avoidance.md`](docs/ultrasonic_avoidance.md) (긴급 회피 로직 설계)
