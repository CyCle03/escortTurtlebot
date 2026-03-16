# escort_turtlebot_pkg

Python launch/bridge integration package for escort simulation and runtime orchestration.

## What It Does
- Launches multi-TurtleBot simulation (`escort_sim.launch.py`).
- Spawns robots with namespaced frames (`{leader_name}`, `{follower_name}`).
- Runs a hybrid follower behavior:
  - Leader rear-target idea from `team_project`
  - Nav2 `FollowPath` execution from `escort_follower`
- Integrates SLAM for the Leader robot:
  - Leader (`{leader_name}`) runs `slam_toolbox` to generate a map of the environment.
  - Follower (`{follower_name}`) uses LiDAR solely for local obstacle avoidance via Nav2 local costmap.
- Automatic Follower Detection (`follower_detector_node`):
  - Dynamically calculates the follower's initial position and relative pose using ICP (Iterative Closest Point) algorithm on LiDAR scans.
  - Synchronizes the `/{leader_name}/odom` and `/{follower_name}/odom` coordinate frames in real-time, eliminating the need for exact initial placement.
  - **Map corruption protection**: rejects ICP corrections that exceed configurable distance/angle limits, skips ICP when follower is stationary (stuck behind obstacle), and preserves the last valid TF when the leader's scan goes stale.
  - All thresholds are tunable via ROS parameters.
- Intelligent Recovery Behavior (`Wait at Last Known Position`):
  - If the leader (`{leader_name}`) is occluded or TF tracking is lost, the follower (`{follower_name}`) cancels its current goal.
  - The follower navigates to the leader's actual last known position. While waiting, it **re-sends the goal periodically** (`recovery_resend_period_sec`) to prevent Nav2 from silently dropping it.

## Nodes Overview

### 1. `follower_detector_node`
Computes the relative position between the leader and follower using ICP scan matching and synchronizes their coordinate frames.

*   **Subscribed Topics:**
    *   `/{leader_name}/scan` (`sensor_msgs/LaserScan`): Leader robot's 2D point cloud data
    *   `/{follower_name}/scan` (`sensor_msgs/LaserScan`): Follower robot's 2D point cloud data
*   **Published TF:**
    *   `/{leader_name}/odom` -> `/{follower_name}/odom`: Aligns follower's odometry frame to the leader's map
*   **Key Parameters:**
    *   `leader_name` (string, default: `TB3_1`): Name of the leader robot.
    *   `follower_name` (string, default: `TB3_2`): Name of the follower robot.
    *   `icp_fitness_threshold` (default `0.3`): Minimum ICP match quality to accept a TF update.
    *   `blend_alpha` (default `0.5`): Blending ratio between previous TF and new ICP result.
    *   `max_correction_dist` (default `0.3` m): ICP result is rejected if it deviates more than this from the current TF (large jump protection).
    *   `max_correction_angle` (default `0.5` rad ≈ 28°): ICP result is rejected if angular deviation exceeds this threshold.
    *   `odom_motion_threshold` (default `0.02` m): If follower's estimated motion is below this value, ICP is skipped entirely (stuck-robot map protection).
    *   `scan_timeout_sec` (default `1.0` s): If no scan is received from leader for this duration, ICP is skipped and the last valid TF is preserved.

### 2. `leader_initial_move_node`
A utility node that moves the leader robot a specified distance at startup. This is useful for creating an initial separation from the follower.

*   **Subscribed Topics:**
    *   `/{leader_name}/odom` (`nav_msgs/Odometry`): Used to track the distance traveled.
*   **Published Topics:**
    *   `/{leader_name}/cmd_vel` (`geometry_msgs/Twist`): Publishes velocity commands to move the robot.
*   **Key Parameters:**
    *   `distance` (double, default: `0.5`): The target distance to move in meters.
    *   `speed` (double, default: `0.10`): The linear speed in m/s.
    *   `startup_delay_sec` (double, default: `2.0`): Delay before the movement starts.
    *   `max_duration_sec` (double, default: `10.0`): Maximum time allowed for the movement.

### Standalone Execution (For Debugging)
To debug TF synchronization or ICP matching issues, you can run the node standalone:
```bash
ros2 run escort_turtlebot_pkg follower_detector_node --ros-args -p leader_name:=<leader> -p follower_name:=<follower>
```

## Detailed Documentation
For deeper dives into the system architecture and configurations, please refer to:
- [System Architecture](docs/architecture.md): Details on the hybrid follower approach, ICP LiDAR matching, and the TF tree.
- [Parameters & Launch](docs/parameters_and_launch.md): Explanations of Nav2 configurations (`dwb_core::DWBLocalPlanner`), local costmap setup, and launch arguments.
- [Leader Initial Move](docs/leader_initial_move.md): Guide to the node for creating initial separation between robots.

## Build
```bash
cd ~/escort_ws/controllserver
colcon build --packages-select escort_turtlebot_pkg escort_follower
source install/setup.bash
```

## Integrated Simulation
```bash
cd ~/escort_ws/controllserver
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source install/setup.bash

ros2 launch escort_turtlebot_pkg escort_sim.launch.py
```

## Leader Keyboard Teleop
Use a separate terminal after simulation starts:

```bash
cd ~/escort_ws/controllserver
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source install/setup.bash

ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=/{leader_name}/cmd_vel
```
(You can change `{leader_name}` via the `leader_name` launch argument.)

## Main Launch Arguments

### `escort_sim.launch.py`
- `use_sim_time` (bool, default: `true`): Use simulation clock.
- `leader_x`, `leader_y` (double): Leader spawn pose.
- `follower_x`, `follower_y` (double): Follower spawn pose.

### `escort_follower.launch.py`
- `use_sim_time` (bool, default: `false`): Use simulation clock.
- `leader_name` (string, default: `TB3_1`): Name of the leader robot.
- `follower_name` (string, default: `TB3_2`): Name of the follower robot.
- `follow_distance` (double, default: `0.5`): Target distance from leader.
- `initial_step_distance` (double, default: `0.0`): One-time initial forward step.

## 한국어 안내

이 패키지는 에스코트 시뮬레이션 실행을 통합하는 Python launch/bridge 패키지입니다.

### 주요 기능
- 멀티 터틀봇 시뮬레이션 실행 (`escort_sim.launch.py`)
- 네임스페이스 기반 프레임(`{leader_name}`, `{follower_name}`)으로 로봇 스폰
- 하이브리드 추종 방식 사용
  - `team_project`의 리더 후방 목표점 아이디어
  - `escort_follower`의 Nav2 `FollowPath` 추종 실행
- 리더 로봇 중심의 SLAM 통합
  - 리더 로봇(`{leader_name}`)은 `slam_toolbox`를 실행하여 맵 생성
  - 팔로워 로봇(`{follower_name}`)은 무거운 SLAM 대신 자체 LiDAR 데이터를 활용한 근거리 역동적 장애물 회피(Local Costmap)만 수행
- ICP 스캔 매칭 기반 실시간 위치 추적 (`follower_detector_node`)
  - 리더(`{leader_name}`)와 팔로워(`{follower_name}`)의 LiDAR 스캔(`LaserScan`) 데이터를 ICP 알고리즘으로 매칭하여 두 로봇 간의 상대 위치를 실시간으로 정밀하게 계산
  - 시작 시 정확한 위치에 로봇을 배치할 필요가 없으며, 지속적으로 `/{follower_name}/odom` 좌표계를 `/{leader_name}/odom` 기준으로 보정하여 위치 오차를 줄임
  - **맵 오염 방지**: ICP 보정량이 허용 범위를 초과하거나, 팔로워가 정지 상태(장애물에 막힘)이거나, 리더 스캔이 stale이면 ICP를 자동으로 건너뛰어 Bridge TF를 보호함
  - 모든 임계값을 ROS 파라미터로 조정 가능
- 지능형 예외 상황 복구 행동 (`Wait at Last Known Position`)
  - 통신 단절이나 장애물 가림(Occlusion) 등으로 리더 로봇(`{leader_name}`)의 위치 정보(TF)를 놓칠 경우, 팔로워(`{follower_name}`)는 즉시 추종을 멈춥니다.
  - 리더 로봇이 마지막으로 목격된 위치로 이동하여 대기하며, **`recovery_resend_period_sec` 주기마다 목표를 재전송**하여 리더가 다시 발견될 때까지 안정적으로 대기합니다.

### 주요 노드 (Nodes Overview)

#### 1. `follower_detector_node`
LiDAR 데이터 기반의 ICP 정합 알고리즘을 사용해 리더와 팔로워 간의 상대 위치를 계산하고 좌표계를 동기화합니다.

*   **Subscribed Topics:**
    *   `/{leader_name}/scan` (`sensor_msgs/LaserScan`): 리더 로봇의 2D 점군 데이터
    *   `/{follower_name}/scan` (`sensor_msgs/LaserScan`): 팔로워 로봇의 2D 점군 데이터
*   **Published TF:**
    *   `/{leader_name}/odom` -> `/{follower_name}/odom`: 팔로워 로봇의 주행 기록(odom) 좌표계를 리더 로봇 기준 통일 맵 상으로 정렬
*   **주요 파라미터:**
    *   `leader_name` (string, 기본값: `TB3_1`): 리더 로봇의 이름.
    *   `follower_name` (string, 기본값: `TB3_2`): 팔로워 로봇의 이름.
    *   `icp_fitness_threshold` (기본값 `0.3`): TF 업데이트를 허용하는 최소 ICP 품질 점수
    *   `blend_alpha` (기본값 `0.5`): 이전 TF와 새 ICP 결과의 혼합 비율
    *   `max_correction_dist` (기본값 `0.3` m): ICP 결과가 현재 TF와 이 거리 이상 차이 나면 reject (급격한 TF 점프 차단)
    *   `max_correction_angle` (기본값 `0.5` rad ≈ 28°): ICP 결과의 각도 편차가 이 값 초과 시 reject
    *   `odom_motion_threshold` (기본값 `0.02` m): 팔로워 추정 이동량이 이 값 이하면 ICP 건너뜀 (장애물에 막힌 상황에서 맵 오염 차단)
    *   `scan_timeout_sec` (기본값 `1.0` s): 리더 스캔이 이 시간 이상 수신되지 않으면 ICP 건너뛰고 마지막 유효 TF 보존

#### 2. `leader_initial_move_node`
시작 시 리더 로봇을 지정된 거리만큼 움직이는 유틸리티 노드입니다. 팔로워와의 초기 거리를 확보하는 데 유용합니다.

*   **Subscribed Topics:**
    *   `/{leader_name}/odom` (`nav_msgs/Odometry`): 이동 거리를 추적하는 데 사용됩니다.
*   **Published Topics:**
    *   `/{leader_name}/cmd_vel` (`geometry_msgs/Twist`): 로봇을 움직이기 위한 속도 명령을 발행합니다.
*   **주요 파라미터:**
    *   `distance` (double, 기본값: `0.5`): 목표 이동 거리 (미터).
    *   `speed` (double, 기본값: `0.10`): 선형 속도 (m/s).
    *   `startup_delay_sec` (double, 기본값: `2.0`): 움직임이 시작되기 전의 지연 시간.
    *   `max_duration_sec` (double, 기본값: `10.0`): 움직임에 허용되는 최대 시간.

#### Follower Detector 단독 실행 (디버깅)
TF 동기화나 ICP 매칭 문제 디버깅 시, 노드만 단독으로 재실행할 수 있습니다.
```bash
ros2 run escort_turtlebot_pkg follower_detector_node --ros-args -p leader_name:=<리더이름> -p follower_name:=<팔로워이름>
```

### 상세 문서
시스템 구조와 파라미터 설정에 대한 더 자세한 내용은 아래 문서를 참고하세요:
- [시스템 및 TF 아키텍처](docs/architecture.md) (영문): 하이브리드 추종 방식, ICP 스캔 매칭, 그리고 TF 트리에 대한 상세 설명
- [파라미터 및 런치 설정](docs/parameters_and_launch.md) (영문): Nav2(`DWBLocalPlanner`), 로컬 코스트맵 설정 및 런치 인자에 대한 설명
- [리더 초기 이동](docs/leader_initial_move.md) (영문): 로봇 간 초기 거리를 생성하는 노드에 대한 안내

### 빌드
```bash
cd ~/escort_ws/controllserver
colcon build --packages-select escort_turtlebot_pkg escort_follower
source install/setup.bash
```

### 통합 시뮬레이션 실행
```bash
cd ~/escort_ws/controllserver
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source install/setup.bash

ros2 launch escort_turtlebot_pkg escort_sim.launch.py
```

### 리더 키보드 조종
시뮬레이션 실행 후, 별도 터미널에서 아래 명령으로 리더(`{leader_name}`)를 조종합니다.

```bash
cd ~/escort_ws/controllserver
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source install/setup.bash

ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=/{leader_name}/cmd_vel
```
(`leader_name`은 런치 인자를 통해 변경할 수 있습니다.)

### 주요 런치 인자

#### `escort_sim.launch.py`
- `use_sim_time` (bool, 기본값 `true`): 시뮬레이션 시간 사용 여부
- `leader_x`, `leader_y`: leader 스폰 위치
- `follower_x`, `follower_y`: follower 스폰 위치

#### `escort_follower.launch.py`
- `use_sim_time` (bool, 기본값 `false`): 시뮬레이션 시간 사용 여부
- `leader_name` (string, 기본값 `TB3_1`): 리더 로봇 이름.
- `follower_name` (string, 기본값 `TB3_2`): 팔로워 로봇 이름.
- `follow_distance` (double, 기본값 `0.5`): 리더와의 목표 추종 거리.
- `initial_step_distance` (double, 기본값 `0.0`): 최초 1회 목표 방향으로 전진할 거리.
