# escort_turtlebot_pkg

Python launch/bridge integration package for escort simulation and runtime orchestration.

## What It Does
- Launches multi-TurtleBot simulation (`escort_sim.launch.py`).
- Spawns robots with namespaced frames (`TB3_1`, `TB3_2`).
- Runs a hybrid follower behavior:
  - Leader rear-target idea from `team_project`
  - Nav2 `FollowPath` execution from `escort_follower`
- Integrates SLAM for the Leader robot:
  - Leader (`TB3_1`) runs `slam_toolbox` to generate a map of the environment.
  - Follower (`TB3_2`) uses LiDAR solely for local obstacle avoidance via Nav2 local costmap.
- Automatic Follower Detection (`follower_detector_node`):
  - Dynamically calculates the follower's initial position and relative pose using ICP (Iterative Closest Point) algorithm on LiDAR scans.
  - Synchronizes the `/TB3_1/odom` and `/TB3_2/odom` coordinate frames in real-time, eliminating the need for exact initial placement.
- Intelligent Recovery Behavior (`Wait at Last Known Position`):
  - If the leader (`TB3_1`) is occluded or TF tracking is lost, the follower (`TB3_2`) cancels its current goal.
  - The follower navigates to the leader's actual last known position and waits until TF tracking is restored, avoiding erratic maneuvers.

## Nodes Overview

### 1. `follower_detector_node`
Computes the relative position between the leader and follower using ICP scan matching and synchronizes their coordinate frames.

*   **Subscribed Topics:**
    *   `/TB3_1/scan` (`sensor_msgs/LaserScan`): Leader robot's 2D point cloud data
    *   `/TB3_2/scan` (`sensor_msgs/LaserScan`): Follower robot's 2D point cloud data
*   **Published TF:**
    *   `TB3_1/odom` -> `TB3_2/odom`: Aligns follower's odometry frame to the leader's map
*   **Required TF Trees:**
    *   `TB3_1/odom` -> `TB3_1/base_scan`
    *   `TB3_2/odom` -> `TB3_2/base_scan`

### Standalone Execution (For Debugging)
To debug TF synchronization or ICP matching issues, you can run the node standalone:
```bash
ros2 run escort_turtlebot_pkg follower_detector_node
```

## Detailed Documentation
For deeper dives into the system architecture and configurations, please refer to:
- [System Architecture](docs/architecture.md): Details on the hybrid follower approach, ICP LiDAR matching, and the TF tree.
- [Parameters & Launch](docs/parameters_and_launch.md): Explanations of Nav2 configurations (`dwb_core::DWBLocalPlanner`), local costmap setup, and launch arguments.

## Build
```bash
cd ~/escort_ws/controllserver
colcon build --packages-select escort_turtlebot_pkg
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

ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=/TB3_1/cmd_vel
```

If your teleop package exposes a different executable name, use that package's command
but keep remapping to `/TB3_1/cmd_vel`.

## Main Launch Arguments (`escort_sim.launch.py`)
- `use_sim_time` (bool, default: `true`)
  Use simulation clock.
- `leader_x`, `leader_y` (double)
  Leader spawn pose.
- `follower_x`, `follower_y` (double)
  Follower spawn pose.

## 한국어 안내

이 패키지는 에스코트 시뮬레이션 실행을 통합하는 Python launch/bridge 패키지입니다.

### 주요 기능
- 멀티 터틀봇 시뮬레이션 실행 (`escort_sim.launch.py`)
- 네임스페이스 기반 프레임(`TB3_1`, `TB3_2`)으로 로봇 스폰
- 하이브리드 추종 방식 사용
  - `team_project`의 리더 후방 목표점 아이디어
  - `escort_follower`의 Nav2 `FollowPath` 추종 실행
- 리더 로봇 중심의 SLAM 통합
  - 리더 로봇(`TB3_1`)은 `slam_toolbox`를 실행하여 맵 생성
  - 팔로워 로봇(`TB3_2`)은 무거운 SLAM 대신 자체 LiDAR 데이터를 활용한 근거리 역동적 장애물 회피(Local Costmap)만 수행
- ICP 스캔 매칭 기반 실시간 위치 추적 (`follower_detector_node`)
  - 리더(`TB3_1`)와 팔로워(`TB3_2`)의 LiDAR 스캔(`LaserScan`) 데이터를 ICP 알고리즘으로 매칭하여 두 로봇 간의 상대 위치를 실시간으로 정밀하게 계산
  - 시작 시 정확한 위치에 로봇을 배치할 필요가 없으며, 지속적으로 `TB3_2/odom` 좌표계를 `TB3_1/odom` 보정하여 위치 오차를 줄임
- 지능형 예외 상황 복구 행동 (`Wait at Last Known Position`)
  - 통신 단절이나 장애물 가림(Occlusion) 등으로 리더 로봇(`TB3_1`)의 위치 정보(TF)를 놓칠 경우, 팔로워(`TB3_2`)는 즉시 추종을 멈춥니다.
  - 이후 리더 로봇이 마지막으로 목격되었던 정확한 위치로 이동하여 대기하다가, 리더가 다시 발견되면 자동으로 에스코트 주행을 재개합니다.

### 주요 노드 (Nodes Overview)

#### 1. `follower_detector_node`
LiDAR 데이터 기반의 ICP 정합 알고리즘을 사용해 리더와 팔로워 간의 상대 위치를 계산하고 좌표계를 동기화합니다.

*   **Subscribed Topics:**
    *   `/TB3_1/scan` (`sensor_msgs/LaserScan`): 리더 로봇의 2D 점군 데이터
    *   `/TB3_2/scan` (`sensor_msgs/LaserScan`): 팔로워 로봇의 2D 점군 데이터
*   **Published TF:**
    *   `TB3_1/odom` -> `TB3_2/odom`: 팔로워 로봇의 주행 기록(odom) 좌표계를 리더 로봇 기준 통일 맵 상으로 정렬
*   **Required TF Trees:**
    *   `TB3_1/odom` -> `TB3_1/base_scan`
    *   `TB3_2/odom` -> `TB3_2/base_scan`

#### Follower Detector 단독 실행 (디버깅)
TF 동기화나 ICP 매칭 문제 디버깅 시, 노드만 단독으로 재실행할 수 있습니다.
```bash
ros2 run escort_turtlebot_pkg follower_detector_node
```

### 상세 문서
시스템 구조와 파라미터 설정에 대한 더 자세한 내용은 아래 문서를 참고하세요:
- [시스템 및 TF 아키텍처](docs/architecture.md) (영문): 하이브리드 추종 방식, ICP 스캔 매칭, 그리고 TF 트리에 대한 상세 설명
- [파라미터 및 런치 설정](docs/parameters_and_launch.md) (영문): Nav2(`DWBLocalPlanner`), 로컬 코스트맵 설정 및 런치 인자에 대한 설명

### 빌드
```bash
cd ~/escort_ws/controllserver
colcon build --packages-select escort_turtlebot_pkg
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
시뮬레이션 실행 후, 별도 터미널에서 아래 명령으로 리더(`TB3_1`)를 조종합니다.

```bash
cd ~/escort_ws/controllserver
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source install/setup.bash

ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=/TB3_1/cmd_vel
```

사용 중인 teleop 패키지의 실행 파일 이름이 다르면 해당 명령을 사용하되,
`cmd_vel` 리매핑은 `/TB3_1/cmd_vel`로 유지하세요.

### 주요 런치 인자 (`escort_sim.launch.py`)
- `use_sim_time` (기본값 `true`): 시뮬레이션 시간 사용 여부
- `leader_x`, `leader_y`: leader 스폰 위치
- `follower_x`, `follower_y`: follower 스폰 위치
