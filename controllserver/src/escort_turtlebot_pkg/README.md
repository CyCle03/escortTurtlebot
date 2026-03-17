# escort_turtlebot_pkg

Python package for launch/bridge integration.

This package is responsible for:
- Launching the simulation environment.
- Managing launch files for the entire system.
- Bridging TF between the robots.
- Vision processing for gesture recognition and mask detection.
- A GUI for system monitoring.

## How to Build

This package is part of the `controllserver` workspace. Build the entire workspace from the root directory (`~/escort_ws`):

```bash
cd ~/escort_ws
colcon build --packages-up-to escort_turtlebot_pkg
source install/setup.bash
```

## How to Run

This package contains the main launch file to start the system:

```bash
ros2 launch escort_turtlebot_pkg escort_full_system.launch.py
```

---

# 한국어 안내

`escort_turtlebot_pkg`는 파이썬으로 작성된 ROS 2 패키지로, 전체 시스템의 런치(실행) 관리, 로봇 간의 TF 브리징, 비전 처리, 그리고 GUI를 담당합니다.

### 주요 기능
- 시뮬레이션 환경 실행 및 관리
- 전체 시스템을 위한 런치 파일 통합
- 리더와 팔로워 로봇 간의 TF 좌표계 연결
- 제스처 인식 및 마스크 감지를 위한 비전 노드
- 시스템 모니터링을 위한 GUI 제공

## 빌드 방법

이 패키지는 `controllserver` 워크스페이스의 일부입니다. 워크스페이스의 루트 디렉터리(`~/escort_ws`)에서 전체 워크스페이스를 빌드하세요.

```bash
cd ~/escort_ws
colcon build --packages-up-to escort_turtlebot_pkg
source install/setup.bash
```

## 실행 방법

이 패키지는 전체 시스템을 시작하는 메인 런치 파일을 포함하고 있습니다.

```bash
ros2 launch escort_turtlebot_pkg escort_full_system.launch.py
```
