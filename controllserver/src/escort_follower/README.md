# escort_follower

C++ package for the follower robot.

This package contains the core algorithms for the follower robot to track the leader. It calculates the leader's position and generates a hybrid tracking path.

## How to Build

This package is part of the `controllserver` workspace. Build the entire workspace from the root directory (`~/escort_ws`):

```bash
cd ~/escort_ws
colcon build --packages-up-to escort_follower
source install/setup.bash
```

## How to Run

Use the launch files in `escort_turtlebot_pkg` to run the system.

---

# 한국어 안내

`escort_follower`는 C++로 작성된 팔로워 로봇의 핵심 패키지입니다.

이 패키지는 리더 로봇을 추종하기 위한 핵심 알고리즘을 포함하고 있습니다. 리더의 위치를 계산하고, 하이브리드 추종 경로를 생성합니다.

## 빌드 방법

이 패키지는 `controllserver` 워크스페이스의 일부입니다. 워크스페이스의 루트 디렉터리(`~/escort_ws`)에서 전체 워크스페이스를 빌드하세요.

```bash
cd ~/escort_ws
colcon build --packages-up-to escort_follower
source install/setup.bash
```

## 실행 방법

`escort_turtlebot_pkg`에 있는 런치 파일을 사용하여 시스템을 실행하세요.
