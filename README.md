# escortTurtlebot

Shared ROS 2 workspace for multi-member TurtleBot projects focusing on leader-follower orchestration, SLAM integration, and autonomous navigation.

## Key Features
- **Hybrid Leader-Follower**: Combines virtual target projection with Nav2 paths.
- **ICP-based TF Alignment**: Real-time synchronization of multi-robot coordinate frames using LiDAR scan matching.
- **Robust SLAM**: Enhanced stability filters to protect map integrity during tracking.
- **Integrated Safety**: Ultrasonic emergency stop and recovery behaviors.

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

## Common Build (Workspace)
```bash
cd ~/escort_ws/controllserver
colcon build
source install/setup.bash
```

---

## 한국어 안내

이 워크스페이스는 터틀봇3를 이용한 리더-팔로워 주행, SLAM 통합 및 자율 주행 프로토타입을 포함합니다.

### 주요 기능
- **하이브리드 리더-팔로워**: 가상 타겟 투사와 Nav2 경로 실행의 결합
- **ICP 기반 TF 정렬**: LiDAR 스캔 매칭을 통한 실시간 좌표계 동기화
- **강인한 SLAM**: 추적 중 맵 오염을 방지하기 위한 안정화 필터 적용
- **통합 안전 시스템**: 초음파 기반 긴급 정지 및 복구 행동

- 루트 `README.md`: 공통 규칙, 패키지 목록, 전체 빌드 방법
- 각 패키지 README: 해당 모듈의 실행 방법, 파라미터, 토픽/TF 등 상세 내용

현재 `escort_follower` 상세 문서는 아래에 있습니다.
- `controllserver/src/escort_follower/README.md`

현재 `escort_turtlebot_pkg` 상세 문서는 아래에 있습니다.
- `controllserver/src/escort_turtlebot_pkg/README.md`
