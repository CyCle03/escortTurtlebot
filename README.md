# escortTurtlebot

Shared ROS 2 workspace for multi-member TurtleBot projects.

## Repository Guide
- Root `README.md` contains project-wide conventions and package index.
- Each package should keep its own detailed README for build/run/config.

## Package Index
- `controllserver/src/escort_follower`  
  C++ follower node package.  
  See module docs: `controllserver/src/escort_follower/README.md`
- `controllserver/src/escort_turtlebot_pkg`  
  Python launch/bridge integration package.  
  Add and maintain package-specific docs inside this folder.

## Common Build (Workspace)
```bash
cd ~/escort_ws/controllserver
colcon build
source install/setup.bash
```

## Integrated Simulation (robot + controllserver)
`escort_sim.launch.py` now supports two follower stacks:
- `team_project`: simple Python leader/follower follower logic
- `escort_follower`: existing Nav2-based follower stack

```bash
cd ~/escort_ws/controllserver
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source install/setup.bash

# Default: team_project-style follower
ros2 launch escort_turtlebot_pkg escort_sim.launch.py follow_stack:=team_project

# Existing controllserver follower stack
ros2 launch escort_turtlebot_pkg escort_sim.launch.py follow_stack:=escort_follower
```

## 한국어 안내

여기는 팀 공용 워크스페이스 인덱스 문서입니다.

- 루트 `README.md`: 공통 규칙, 패키지 목록, 전체 빌드 방법
- 각 패키지 README: 해당 모듈의 실행 방법, 파라미터, 토픽/TF 등 상세 내용

현재 `escort_follower` 상세 문서는 아래에 있습니다.
- `controllserver/src/escort_follower/README.md`

### 통합 시뮬레이션 (robot + controllserver)
`escort_sim.launch.py`는 follower 스택을 2가지로 선택할 수 있습니다.
- `team_project`: Python 기반 leader/follower 추종 로직
- `escort_follower`: 기존 Nav2 기반 follower 스택

```bash
cd ~/escort_ws/controllserver
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source install/setup.bash

# 기본값: team_project 추종 방식
ros2 launch escort_turtlebot_pkg escort_sim.launch.py follow_stack:=team_project

# 기존 controllserver follower 스택
ros2 launch escort_turtlebot_pkg escort_sim.launch.py follow_stack:=escort_follower
```
