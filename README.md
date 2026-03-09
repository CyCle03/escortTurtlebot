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
cd /home/penguin/escort_ws/controllserver
colcon build
source install/setup.bash
```

## 한국어 안내

여기는 팀 공용 워크스페이스 인덱스 문서입니다.

- 루트 `README.md`: 공통 규칙, 패키지 목록, 전체 빌드 방법
- 각 패키지 README: 해당 모듈의 실행 방법, 파라미터, 토픽/TF 등 상세 내용

현재 `escort_follower` 상세 문서는 아래에 있습니다.
- `controllserver/src/escort_follower/README.md`
