# `escort_turtlebot_pkg` vs `escort_follower` 역할 및 차이점 비교 가이드

이 문서는 멀티 터틀봇 에스코트 시스템을 구성하는 두 핵심 패키지인 `escort_turtlebot_pkg`와 `escort_follower`의 구조적 역할, 핵심 모듈, 그리고 구체적인 차이점을 명확하게 비교 설명하기 위해 작성되었습니다.

---

## 🏗️ 1. 전체 아키텍처 개요

에스코트 시스템은 두 가지 큰 축으로 나뉩니다.
1. **[제어/통합 환경 계층]**: 전체 시나리오를 구성하고 여러 로봇 노드들을 조율하는 역할 (`escort_turtlebot_pkg`)
2. **[핵심 추종 알고리즘 계층]**: 팔로워 로봇이 리더를 인식하고, 수학적으로 목표점을 산출하여 쫓아가는 역할 (`escort_follower`)

즉, **`escort_turtlebot_pkg`라는 큰 껍데기(인프라) 안에서 `escort_follower`라는 핵심 두뇌(알고리즘)가 동작하는 구조**입니다.

---

## 📦 2. 패키지 상세 분석

### 🅰️ `escort_turtlebot_pkg` (인프라 통합 및 시나리오 런처 패키지)
**"언제, 어떻게, 누구를 실행할 것인가"**를 결정하는 전체 시스템의 매니저 역할을 합니다.

- **사용 언어/빌드**: Python (`ament_python`)
- **주요 역할**:
  - **시뮬레이션 및 실제 구동 통합**: Gazebo 환경 설정, 지도(SLAM) 작성 여부, 로봇 스폰(Spawn) 등 하드웨어/시뮬레이션 의존적인 실행 환경을 세팅합니다.
  - **이름 충돌 방지(Namespace)**: `TB3_1`, `TB3_2` 등 다수 로봇의 TF 트리, 노드 이름, 토픽들이 충돌하지 않도록 격리(Namespace)하고 연결해 줍니다.
  - **시나리오 기획 및 동적 좌표 변환**: "시작 시 리더가 앞으로 0.5m 이동하여 자리를 확보한다"와 같은 고수준의 에스코트 시나리오 처리(`leader_initial_move_node.py`), 그리고 리더의 라이다를 분석해 팔로워의 초기 위치를 자동으로 찾아 TF를 연결해 주는 노드(`follower_detector_node.py`)가 포함되어 있습니다.
  - **안정화 및 복구 로직**: ICP 매칭 시 급격한 보정을 제한하고, 리더를 놓쳤을 때 마지막 위치를 유지하는 등 시스템 전체의 강인성을 담당합니다.
  - **Nav2 시스템 및 SLAM 연동**: `slam_toolbox`, Nav2 모듈들을 불러오고 각 로봇의 파라미터(`escort_controll_server1.yaml`)를 세팅합니다.
- **주요 파일**: 
  - `launch/escort_sim.launch.py`, `escort_real.launch.py` (시스템 런처)
  - `param/escort_controll_server1.yaml` (Nav2/SLAM 파라미터)
  - `escort_turtlebot_pkg/follower_detector_node.py` (라이다 기반 팔로워 자동 인식 노드)

### 🅱️ `escort_follower` (핵심 추종 제어 패키지)
**"리더를 어떻게 인식하고, 어디를 목표로 주행할 것인가"**에 대한 순수 핵심 수식과 알고리즘이 들어있는 패키지입니다.

- **사용 언어/빌드**: C++ (`ament_cmake`)
- **주요 역할**:
  - **리더 좌표계 인식 (TF Lookup)**: `/TB3_1/odom`과 `/TB3_2/odom` 사이의 상대적인 수학적 위치 변환 행렬(TF)을 해독하여 현재 내(팔로워) 위치 대비 리더의 x, y, yaw를 계산합니다.
  - **후방 추종(Targeting) 수학 모델 구현**: 단순히 리더를 쫓아가는 것이 아니라, 리더 진행 방향을 기준으로 '후방 0.5m 지점(`follow_distance`)'이라는 가상의 목표점을 산출합니다.
  - **Nav2 Action 통신 (FollowPath)**: 계산된 목표점을 향해 Nav2의 Action Server(`FollowPath`)로 주행 명령(Goal)을 포맷에 맞게 쏴줍니다. 목표가 너무 적게 변했을 땐 전송을 거르는 통신 최적화 로직도 포함됩니다.
- **주요 파일**: 
  - `src/follower.cpp` (추종 경로 산출 핵심 엔진)

---

## ⚔️ 3. 두 패키지의 직무 차이 요약

| 구분 | `escort_turtlebot_pkg` (런처/통합) | `escort_follower` (C++ 알고리즘) |
| :--- | :--- | :--- |
| **핵심 목적** | 로봇들을 켜고 파라미터를 먹여서 **무대를 준비**함 | 리더 위치를 계산해 **목표 경로를 산출**함 |
| **사용 언어** | Python (ROS2 Launch 중심) | C++ (ROS2 Node 중심 고성능 처리) |
| **의존성(Depend)** | Gazebo, Nav2, SLAM, TF, **`escort_follower`** | TF2 기하학, Nav2 Action Client |
| **수정 빈도** | 로봇 대수 변경, 센서/SLAM 설정, 시나리오(시작/종료) 변경 시 수정 | 리더 추종 '수식' 변경, 목표점 갱신 빈도 등 알고리즘 로직 변경 시 수정 |

**결론적으로:** `escort_turtlebot_pkg`는 프로젝트의 원활한 통합 구동을 담당하는 "매니저"이고, `escort_follower`는 리더를 졸졸 따라가기 위해 계산을 수행하는 전문 "두뇌"로 비유할 수 있습니다. 
