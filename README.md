# escortTurtlebot

ROS 2 Humble workspace for multi-TurtleBot escort/follower control.

## Packages
- `escort_follower` (C++): creates follower nodes and sends `FollowPath` goals from TF-based relative poses.
- `escort_turtlebot_pkg` (Python): launch integration and optional LiDAR-based odom bridge.

## Build
```bash
cd /home/penguin/escort_ws/controllserver
colcon build --packages-select escort_follower escort_turtlebot_pkg
source install/setup.bash
```

## Run (example)
```bash
ros2 launch escort_turtlebot_pkg escort_follower.launch.py \
  number_of_follower:=1 use_lidar_bridge:=true use_sim_time:=false
```

## Key Launch Args
- `number_of_follower`: number of followers (1 to 4)
- `use_lidar_bridge`: `true` for LiDAR odom bridge, `false` for static transform bridge
- `initial_dx`, `initial_dy`: static bridge offsets used when `use_lidar_bridge:=false`

## 한국어 안내

ROS 2 Humble 기반의 다중 TurtleBot 에스코트/추종 제어 워크스페이스입니다.

### 패키지
- `escort_follower` (C++): TF 기반 상대 위치를 사용해 `FollowPath` goal을 생성/전송합니다.
- `escort_turtlebot_pkg` (Python): 런치 통합과 선택형 LiDAR 기반 odom 브리지를 제공합니다.

### 빌드
```bash
cd /home/penguin/escort_ws/controllserver
colcon build --packages-select escort_follower escort_turtlebot_pkg
source install/setup.bash
```

### 실행 예시
```bash
ros2 launch escort_turtlebot_pkg escort_follower.launch.py \
  number_of_follower:=1 use_lidar_bridge:=true use_sim_time:=false
```

### 주요 런치 인자
- `number_of_follower`: follower 로봇 수 (1~4)
- `use_lidar_bridge`: `true`면 LiDAR odom 브리지, `false`면 static transform 브리지 사용
- `initial_dx`, `initial_dy`: `use_lidar_bridge:=false`일 때 사용하는 static 브리지 오프셋
