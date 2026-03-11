# Parameters and Configurations

This document explains the key parameters, launch arguments, and configuration choices used in the **escort_turtlebot_pkg** project to achieve smooth and stable multi-robot escort behavior.

## Core Launch Arguments

The launch files (e.g., `escort_sim.launch.py` and `escort_real.launch.py`) accept several arguments to customize the behavior without recompiling code.

| Argument | Type | Default Value | Description |
|---|---|---|---|
| `use_sim_time` | bool | `False` (Real), `True` (Sim) | **Crucial:** Set to `False` when running on physical TurtleBots. If set incorrectly, TF and sensor messages will freeze waiting for Gazebo's clock. |
| `number_of_follower` | int | 1 | The number of followers. Expandable for convoy setups. |
| `follow_distance` | double | `0.5` | The target distance (in meters) the follower tries to maintain strictly behind the leader. |
| `odom_bridge_x` | double | `-0.5` | The initial presumed offset distance between the leader and follower when the ICP scan matching first kicks in. |

## Network & Middleware Configurations

When deploying on real robots (Raspberry Pi + Control PC), the default ROS 2 `FastRTPS` middleware often suffers from severe UDP packet loss, causing critical topics (like `/TB3_1/odom`) to disappear.

**Recommendation:** Always use **CycloneDDS** for real-world multi-robot deployments.
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp -y
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
Make sure to apply this to **ALL** devices in the network (Leader PI, Follower PI, and the Control PC).

## Follower Node Parameters (`follower.cpp`)

The follower behavior logic exposes parameters to fine-tune how aggressively it tracks the leader.

- `goal_update_distance_threshold` (`0.03` m): The follower will not send a new Nav2 goal if the leader hasn't moved at least 3cm. This prevents rapid-fire goal spamming and jittering.
- `goal_update_min_period_sec` (`0.3` s): The follower will wait at least 0.3 seconds before sending a new target, giving the Nav2 controller enough time to process and accelerate.
- `tf_timeout_sec` (`2.0` s): If the leader's position is not updated via TF for 2.0 seconds (e.g., due to occlusion or network drop), the follower aborts the current tracking and enters **Recovery Mode** (Waiting at the last known position).

## Nav2 Controller Configurations (`param/escort_controll_server1.yaml`)

The `dwb_core::DWBLocalPlanner` on the follower is specifically tuned for smooth tracking rather than aggressive exploration:

- `min_vel_x: -0.12`: Allows the follower to reverse slowly if the leader stops abruptly or if the gap becomes too tight. Setting this to `0.0` would force the follower to spin 180 degrees to back up, which looks unnatural.
- `BaseObstacle.scale: 0.02`: The local costmap obstacle avoidance is set relatively low. Since the follower is closely tailing the leader, if the obstacle repulsion weight is too high, the follower might view the leader itself as an obstacle and refuse to approach it.
- `PathDist.scale: 32.0`: The follower heavily prioritizes sticking to the exact path generated towards the leader.

---

## 한국어 안내 (Parameters and Configurations)

이 문서는 **escort_turtlebot_pkg** 프로젝트에서 안정적인 다중 로봇 에스코트 주행을 달성하기 위해 사용된 핵심 파라미터, 런치 인자, 그리고 주요 설정들에 대해 설명합니다.

### 핵심 런치 인자 (Launch Arguments)

런치 파일들(예: `escort_sim.launch.py` 및 `escort_real.launch.py`)은 코드를 다시 컴파일하지 않고도 로봇의 동작을 제어할 수 있는 여러 인자를 제공합니다.

| 인자 (Argument) | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `use_sim_time` | bool | `False` (실제), `True` (시뮬레이션) | **매우 중요:** 실제 물리적인 터틀봇을 구동할 때는 반드시 `False`로 설정해야 합니다. 잘못 설정하면 로봇이 존재하지 않는 Gazebo 가상 시계를 기다리느라 모든 센서 데이터와 TF가 멈춰버립니다. |
| `number_of_follower` | int | 1 | 팔로워 로봇의 수입니다. 기차(Convoy) 대형으로 확장할 때 사용합니다. |
| `follow_distance` | double | `0.5` | 팔로워가 리더의 바로 뒤쪽에서 유지하려고 하는 목표 추종 거리(미터)입니다. |
| `odom_bridge_x` | double | `-0.5` | 시스템 시작 시, ICP 스캔 매칭이 초기 보정을 수행하기 전에 가정하는 두 로봇 간의 대략적인 오프셋 거리입니다. |

### 네트워크 및 통신 미들웨어 설정

실제 로봇 환경(라즈베리파이 + 제어 PC)에서 배포할 때, 기본 ROS 2 미들웨어인 `FastRTPS`를 사용하면 UDP 패킷 손실이 심각하게 발생하여 필수적인 토픽들(예: `/TB3_1/odom`)이 증발해버리는 현상이 잦습니다.

**권장 사항:** 다중 로봇을 실제 환경에서 구동할 때는 반드시 **CycloneDDS** 미들웨어를 사용하십시오.
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp -y
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
이 설정은 네트워크에 연결된 **모든 기기** (리더 P, 팔로워 Pi, 제어용 노트북 PC)의 `~/.bashrc`에 동일하게 적용되어야 합니다.

### 팔로워 제어 노드 파라미터 (`follower.cpp`)

팔로워가 리더를 얼마나 공격적으로 혹은 부드럽게 따라갈지 결정하는 세부 파라미터들입니다.

- `goal_update_distance_threshold` (`0.03` m): 리더가 최소 3cm 이상 이동하지 않으면 팔로워는 새로운 목표점(Goal)을 전송하지 않습니다. 목표점이 너무 자주 갱신되어 로봇이 파르르 떨리는 현상(Jittering)을 방지합니다.
- `goal_update_min_period_sec` (`0.3` s): 팔로워는 Nav2 제어기가 가속도를 계산하고 반영할 수 있도록, 새로운 목표점을 보내기 전에 최소 0.3초를 대기합니다.
- `tf_timeout_sec` (`2.0` s): 장애물에 가려지거나 네트워크 끊김으로 인해 리더의 위치(TF)를 2.0초 이상 갱신받지 못하면, 팔로워는 즉시 현재 추종을 멈추고 **복구 모드(Recovery Mode)** 로 진입하여 리더가 마지막으로 목격된 위치로 가서 대기합니다.

### Nav2 로컬 제어기 설정 (`param/escort_controll_server1.yaml`)

팔로워 로봇의 `dwb_core::DWBLocalPlanner`는 공격적이고 빠른 자율주행보다는 부드러운 에스코트 주행을 위해 다음과 같이 특별하게 튜닝되었습니다:

- `min_vel_x: -0.12`: 리더가 급정거해서 간격이 너무 좁아졌을 때 팔로워가 천천히 뒤로 후진할 수 있게 해줍니다. 이 값이 `0.0`이면 로봇이 후진을 못 하고 제자리에서 180도 돌아버리는 기괴한 회피 기동을 하게 됩니다.
- `BaseObstacle.scale: 0.02`: 로컬 코스트맵의 장애물 회피 가중치를 의도적으로 아주 낮게 설정했습니다. 팔로워는 항상 리더 뒤에 바짝 붙어있어야 하므로, 장애물 회피 가중치가 너무 높으면 팔로워가 리더 자체를 '커다란 장애물'로 인식하고 다가가는 것을 무서워하게 되기 때문입니다.
- `PathDist.scale: 32.0`: 팔로워가 다른 어떤 요인보다도 생성된 목표점과 정확한 추종 경로(Path)를 밟고 따라가는 것을 최우선으로 여기도록 설정했습니다.
