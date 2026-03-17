# escort_follower

C++ package for the follower robot.

This package contains the core algorithms for the follower robot to track the leader. It calculates the leader's position and generates a hybrid tracking path.

## How to Build

This package is part of the `escort_ws` workspace. Build the workspace from the root directory (`~/escort_ws`):

```bash
cd ~/escort_ws
colcon build --packages-select escort_follower
source install/setup.bash
```

## Additional Documentation
This package implements several sophisticated behaviors which are documented separately for clarity:

- **Distance Measurement**: Logic for verifying distance between leader and follower.
  - See: [docs/distance_measurement_reference.md](docs/distance_measurement_reference.md)
- **Ultrasonic Avoidance & Safety**: Primary collision safety module, integrating hardware sonar with the logical tracking commands.
  - See: [docs/ultrasonic_avoidance.md](docs/ultrasonic_avoidance.md)

---

# 한국어 안내

`escort_follower`는 C++로 작성된 팔로워 로봇의 핵심 패키지입니다.

이 패키지는 리더 로봇을 추종하기 위한 핵심 알고리즘을 포함하고 있습니다. 리더의 위치를 계산하고, 하이브리드 추종 경로를 생성합니다.

## 빌드 방법

워크스페이스의 루트 디렉터리(`~/escort_ws`)에서 빌드를 수행합니다.

```bash
cd ~/escort_ws
colcon build --packages-select escort_follower
source install/setup.bash
```

## 상세 문서 (Additional Documentation)
이 패키지는 리더 추종뿐만 아니라 여러 정교한 행동 패턴을 구현하고 있으며, 세부 내용은 별도 문서로 제공됩니다:

- **거리 측정 스크립트 가이드**: 리더와 팔로워 간의 거리 유지 상태를 확인하는 로직.
  - 문서 링크: [docs/distance_measurement_reference.md](docs/distance_measurement_reference.md)
- **초음파 긴급 회피 및 안전 장치**: 초음파 센서와 논리적 추종 명령을 통합한 핵심 충돌 방지 모듈. 최근에 긴급 후진 명령이 우선순위를 갖도록 수정되었습니다.
  - 문서 링크: [docs/ultrasonic_avoidance.md](docs/ultrasonic_avoidance.md)
