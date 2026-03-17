# escort_turtlebot_pkg

Python package for launch/bridge integration.

This package is responsible for:
- Launching the simulation environment.
- Managing launch files for the entire system.
- Bridging TF between the robots.
- Vision processing for gesture recognition and mask detection.
- A GUI for system monitoring.

## Documentation
For deep dives into how this package coordinates the multi-robot system, please refer to:
- **[System Architecture](docs/architecture.md)**: Details on the hybrid follower approach, ICP LiDAR matching for TF synchronization, and SLAM stability.
- **[Parameters & Launch](docs/parameters_and_launch.md)**: Explanation of launch arguments, Nav2 configurations, and how the hardware SSH execution works.

## How to Build

This package is part of the `escort_ws` workspace. Build the workspace from the root directory:

```bash
cd ~/escort_ws
colcon build --packages-select escort_turtlebot_pkg escort_follower
source install/setup.bash
```

## How to Run

### 1. Full System (Vision + GUI)
Starts the Vision processing node (YOLOv8 + MediaPipe) and the Robot Control GUI.
```bash
ros2 launch escort_turtlebot_pkg escort_full_system.launch.py
```

### 2. Simulation Mode
Starts the Gazebo simulation with both Leader and Follower robots.
```bash
ros2 launch escort_turtlebot_pkg escort_sim.launch.py
```

### 3. Real Robot Mode
Starts the core follower logic and hardware bridges (including remote Sonar via SSH).
```bash
ros2 launch escort_turtlebot_pkg escort_real.launch.py
```

---

# 한국어 안내

`escort_turtlebot_pkg`는 파이썬으로 작성된 ROS 2 패키지로, 전체 시스템의 런치(실행) 관리, 로봇 간의 TF 브리징, 비전 처리, 그리고 GUI를 담당합니다.

### 주요 기능
- 시뮬레이션 환경 실행 및 관리
- 전체 시스템을 위한 런치 파일 통합
- 리더와 팔로워 로봇 간의 TF 좌표계 연결
- 제스처 인식 및 마스크 감지를 위한 비전 노드 (YOLOv8, MediaPipe)
- 시스템 모니터링 및 수동 제어를 위한 GUI 패널

## 상세 문서 (Documentation)
이 패키지가 다중 로봇 시스템을 어떻게 조율하는지에 대한 기술적 세부 사항은 아래 문서를 참조하세요:
- **[시스템 아키텍처 가이드](docs/architecture.md)**: 하이브리드 추종 방식, TF 동기화를 위한 ICP 라이다 매칭 로직, 그리고 SLAM 안정화 방법에 대한 설명.
- **[파라미터 및 런치 설정](docs/parameters_and_launch.md)**: 런치 파일 인자, Nav2 설정 값, 그리고 원격 하드웨어 SSH 실행 방식에 대한 명세.

## 빌드 방법

워크스페이스의 루트 디렉터리(`~/escort_ws`)에서 빌드를 수행합니다.

```bash
cd ~/escort_ws
colcon build --packages-select escort_turtlebot_pkg escort_follower
source install/setup.bash
```

## 실행 방법

### 1. 전체 시스템 실행 (비전 + GUI)
비전 처리 노드와 로봇 제어 GUI를 함께 실행합니다.
```bash
ros2 launch escort_turtlebot_pkg escort_full_system.launch.py
```

### 2. 시뮬레이션 실행
Gazebo 환경에서 리더와 팔로워 로봇을 함께 소환하여 테스트합니다.
```bash
ros2 launch escort_turtlebot_pkg escort_sim.launch.py
```

### 3. 실제 로봇 실행
실제 로봇 구동을 위한 코어 로직과 하드웨어 브리지(SSH 원격 초음파 센서 포함)를 실행합니다.
```bash
ros2 launch escort_turtlebot_pkg escort_real.launch.py
```

