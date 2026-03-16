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
  See module docs: `controllserver/src/escort_turtlebot_pkg/README.md`

## Detailed Documentation
- **[System Architecture](controllserver/src/escort_turtlebot_pkg/docs/architecture.md)**: In-depth explanation of the hybrid follower model, ICP-based localization, and TF tree.
- **[Parameters and Launch Configurations](controllserver/src/escort_turtlebot_pkg/docs/parameters_and_launch.md)**: Detailed guide on parameters, launch arguments, and Nav2/SLAM configurations.
- **[Package Differences](controllserver/docs/package_differences.md)**: Comparison of the roles and responsibilities of `escort_follower` and `escort_turtlebot_pkg`.
- **[Leader Initial Move](controllserver/src/escort_turtlebot_pkg/docs/leader_initial_move.md)**: Guide to the node for creating initial separation between robots.

## Common Build (Workspace)
```bash
cd ~/escort_ws/controllserver
colcon build
source install/setup.bash
```

## 한국어 안내

여기는 팀 공용 워크스페이스 인덱스 문서입니다.

- 루트 `README.md`: 공통 규칙, 패키지 목록, 전체 빌드 방법
- 각 패키지 README: 해당 모듈의 실행 방법, 파라미터, 토픽/TF 등 상세 내용

현재 `escort_follower` 상세 문서는 아래에 있습니다.
- `controllserver/src/escort_follower/README.md`

현재 `escort_turtlebot_pkg` 상세 문서는 아래에 있습니다.
- `controllserver/src/escort_turtlebot_pkg/README.md`

## 상세 문서
- **[시스템 아키텍처](controllserver/src/escort_turtlebot_pkg/docs/architecture.md)**: 하이브리드 팔로워 모델, ICP 기반 위치 추정, TF 트리에 대한 심층 설명
- **[파라미터 및 실행 설정](controllserver/src/escort_turtlebot_pkg/docs/parameters_and_launch.md)**: 파라미터, 런치 인자, Nav2/SLAM 설정에 대한 상세 가이드
- **[패키지 역할 비교](controllserver/docs/package_differences.md)**: `escort_follower`와 `escort_turtlebot_pkg`의 역할과 책임 비교
- **[리더 초기 이동](controllserver/src/escort_turtlebot_pkg/docs/leader_initial_move.md)**: 로봇 간 초기 거리를 생성하는 노드에 대한 안내
