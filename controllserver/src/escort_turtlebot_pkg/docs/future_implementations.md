# Future Implementations

This document tracks potential features and enhancements categorized for future development of the `escort_turtlebot_pkg`.

## [Planned] Dynamic Distance Control based on Leader Velocity
Currently, the follower maintains a static following distance (e.g., `0.5m`).
- **Goal:** Adapt the `follow_distance` dynamically based on the leader's actual speed.
- **Logic:**
  - If the leader speeds up, increase the follow distance (e.g., to `1.0m+`) to allow for a safer braking distance.
  - If the leader slows down or stops, decrease the follow distance (e.g., back to `0.3m`) to maintain a tight formation.
- **Implementation Ideas:** Subscribe to `/TB3_1/cmd_vel` or calculate velocity from TF deltas over time in `follower_detector_node` or `follower.cpp`, and adjust the target offset dynamically before sending the Nav2 goal.

## [Idea] Multi-Robot Convoy Expansion (TB3_3+)
- **Goal:** Extend the system to support a train (convoy) of robots where TB3_2 follows TB3_1, and TB3_3 follows TB3_2.
- **Logic:** Add support for dynamic namespace scaling in the launch files and ensure `follower_detector_node` can handle an arbitrary number of ICP pairs (e.g., `TB3_2` detecting `TB3_3`).

## [Idea] Automated Deployment Scripts
- **Goal:** Reduce the setup time on physical robots.
- **Logic:** Create bash scripts (`setup_leader.sh`, `setup_follower.sh`) that automatically configure timezone (`chrony`), DDS middleware (`CycloneDDS`), and environmental variables (`LDS_MODEL`) on the Raspberry Pis.

---

## 한국어 안내 (Future Implementations)

이 문서는 `escort_turtlebot_pkg`의 향후 개발 및 개선 아이디어 목록을 관리합니다.

### 1. [예정] 리더 속도 기반 '동적 간격 제어' 알고리즘
현재 팔로워는 고정된 추종 거리(`0.5m`)를 유지합니다.
- **목표:** 리더의 실제 주행 속도에 맞춰 `follow_distance`를 동적으로 조절합니다.
- **로직:**
  - 리더가 속도를 높이면 넉넉한 제동 거리를 위해 추종 간션을 넓힙니다 (예: `1.0m` 이상).
  - 리더가 속도를 줄이거나 멈추면 대열 유지를 위해 간격을 좁힙니다 (예: `0.3m`).
- **구현 아이디어:** `/TB3_1/cmd_vel`을 구독하거나 `follower_detector_node`에서 계산되는 TF 이동량(속도)을 활용하여, Nav2 목표점을 전송하기 전에 목표 오프셋 위치를 실시간으로 재계산합니다.

### 2. [아이디어] 다중 로봇 편대 주행 확장 (TB3_3 추가)
- **목표:** TB3_2가 TB3_1을 따라가고, TB3_3이 다시 TB3_2를 따라가는 기차(Convoy) 형태의 다중 로봇 트랙킹 시스템을 구축합니다.
- **로직:** 런치 파일에서 네임스페이스를 동적으로 확장할 수 있게 만들고, `follower_detector_node`가 여러 쌍의 로봇(예: `TB3_2`와 `TB3_3`) 사이에서도 독립적으로 ICP 매칭을 병렬 수행하도록 구조를 확장합니다.

### 3. [아이디어] 로봇 셋업 및 배포 자동화 스크립트 도구
- **목표:** 새로운 라즈베리파이(로봇) 세팅에 들어가는 고된 작업 시간 단축.
- **로직:** 단 한 줄의 터미널 명령어로 시간 동기화(`chrony`), 통신 미들웨어(`CycloneDDS`), 라이다 기종 변수(`LDS_MODEL`), 그리고 패키지 빌드까지 한 번에 완료해 주는 Bash 스크립트(`setup_leader.sh`, `setup_follower.sh`)를 제작합니다.
