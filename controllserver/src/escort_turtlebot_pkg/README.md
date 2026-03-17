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
  - Synchronizes the `/TB3_1/odom` and `/TB3_2/odom` coordinate frames in real-time, eliminating the need for exact initial placement.
- **Enhanced Safety**: Follower now supports ultrasonic-based emergency obstacle avoidance.
- **Improved Stability**: ICP matching now includes motion-based filtering and correction limits to prevent map wobble.
- **Recovery Behavior**: If the leader is lost or scanned data is stale, the follower waits at its last known relative position by preserving the TF bridge.

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

## Troubleshooting

### Map Wobble / SLAM Corruption
If the map in Rviz appears to wobble or "jump":
- **Cause**: ICP matching might be correcting based on poor scan data (e.g., when the follower is stuck or in a featureless hallway).
- **Solution**: The system now includes `odom_motion_threshold` to skip ICP when the robot is stationary and `max_correction_dist` to reject large, suspicious jumps. Check if `use_sim_time` is correctly set.

### TF Lookup Failures
- **Symptoms**: Follower stops moving or warnings appear in the console.
- **Solution**: Ensure all namespaces (`TB3_1`, `TB3_2`) are correctly applied. The `follower_detector_node` requires both robots' LiDAR scans and their respective `odom` frames to be active.

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
  - 리더 로봇(`TB3_1`)은 `slam_toolbox`를 실행하며 주변 환경의 전체 지도를 생성합니다.
  - 팔로워 로봇(`TB3_2`)은 무거운 SLAM을 동시에 구동하는 대신, 자체 LiDAR 데이터를 활용해 Nav2 로컬 코스트맵(Local Costmap)에서만 실시간 장애물 회피를 수행하여 연산 효율을 높였습니다.
  - 리더와 팔로워의 오도메트리(`odom`) 좌표계를 실시간으로 동기화하므로, 시작 시 두 로봇을 정확한 위치에 수동으로 배치할 필요가 없습니다.
- **안전성 및 복구 기능**:
  - **긴급 회피**: 초음파 센서를 활용해 10cm 이내 장애물 감지 시 즉시 안전 후진을 수행합니다.
  - **안정화 필터**: ICP 보정치 제한 및 이동 기반 필터링을 통해 맵이 튀거나 흔들리는 현상을 방지합니다.
  - **복구 모드**: 리더봇의 신호를 놓쳤을 때 마지막으로 확인된 상대 위치를 유지하며 안정적으로 대기합니다 (Wait at Last Known Position).

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

### 트러블슈팅 (Troubleshooting)

#### 맵 흔들림 (Map Wobble) / SLAM 오염
Rviz 상에서 맵이나 로봇 위치가 급격히 튀는 경우:
- **원인**: 부적절한 ICP 매칭 결과가 TF에 반영되었을 가능성이 큼.
- **해결**: 현재 시스템에는 `odom_motion_threshold`(정지 상태 시 ICP 건너뜀)와 `max_correction_dist`(급격한 위치 변화 거부) 로직이 포함되어 있습니다. `use_sim_time` 설정이 환경에 맞게 되어있는지 확인하세요.

#### TF 조회 실패
- **증상**: 팔로워가 움직이지 않거나 콘솔에 경고 발생.
- **해결**: 네임스페이스(`TB3_1`, `TB3_2`)가 정확히 설정되었는지 확인하세요. `follower_detector_node`가 실행되려면 두 로봇의 LiDAR 스캔과 각각의 `odom` 프레임이 활성화되어 있어야 합니다.

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
