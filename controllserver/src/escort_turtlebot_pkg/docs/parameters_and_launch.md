# Parameters and Configurations

This document explains the key parameters, launch arguments, and configuration choices used in the **escort_turtlebot_pkg** project to achieve smooth and stable multi-robot escort behavior.

## Core Launch Arguments

The launch files (e.g., `escort_sim.launch.py` and `escort_real.launch.py`) accept several arguments to customize the behavior without recompiling code.

| Argument | Type | Default Value | Description |
|---|---|---|---|
| `use_sim_time` | bool | `False` (Real), `True` (Sim) | **Crucial:** Set to `False` when running on physical TurtleBots. If set incorrectly, TF and sensor messages will freeze waiting for Gazebo's clock. |
| `number_of_follower` | int | 1 | The number of followers. Expandable for convoy setups. |
| `follow_distance` | double | `0.5` | The target distance (in meters) the follower tries to maintain strictly behind the leader. |

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
- `tf_timeout_sec` (`2.0` s): If the leader's position is not updated via TF for 2.0 seconds (e.g., due to occlusion or network drop), the follower enters **Recovery Mode**.
- `recovery_resend_period_sec` (`3.0` s): While in Recovery Mode, the "go to last known position" goal is **re-sent every 3 seconds**. This prevents the follower from stopping if Nav2 internally drops a goal after completing it or failing silently.

## Nav2 Controller Configurations (`escort_follower/param/controll_server1.yaml`)

The `dwb_core::DWBLocalPlanner` on the follower is specifically tuned for smooth tracking rather than aggressive exploration:

- `min_vel_x: -0.12`: Allows the follower to reverse slowly if the leader stops abruptly or if the gap becomes too tight. Setting this to `0.0` would force the follower to spin 180 degrees to back up, which looks unnatural.
- `BaseObstacle.scale: 0.02`: The local costmap obstacle avoidance is set relatively low. Since the follower is closely tailing the leader, if the obstacle repulsion weight is too high, the follower might view the leader itself as an obstacle and refuse to approach it.
- `PathDist.scale: 32.0`: The follower heavily prioritizes sticking to the exact path generated towards the leader.

---

## Follower Detector Node Parameters (`follower_detector_node.py`)

The `follower_detector_node` performs ICP (Iterative Closest Point) scan matching to synchronize the `TB3_1/odom` and `TB3_2/odom` coordinate frames in real-time.

### ICP Quality & Blending

- `icp_fitness_threshold` (`0.3`): Minimum ICP match quality score (0–1) required to accept a new TF estimate. Raise this value if false-positive matches cause unstable tracking; lower it in low-feature environments. Default was `0.15`, raised to `0.3` for better robustness in real-world use.
- `blend_alpha` (`0.5`): Blending ratio between the previous TF estimate and the new ICP result. `0.0` means no update (keep old), `1.0` means fully adopt the new ICP result. Increase toward `0.7` for faster responsiveness; decrease toward `0.3` for smoother filtering.

### Map Corruption Protection

These parameters guard against map instability when TB3_2 gets stuck or the leader disappears:

- `max_correction_dist` (`0.3` m): If the new ICP result deviates more than this distance from the current bridge TF, it is **rejected** as a likely false match. Prevents sudden large TF jumps from corrupting the SLAM map.
- `max_correction_angle` (`0.5` rad ≈ 28°): Same as above, but for angular deviation. Keeps the bridge TF from rotating wildly.
- `odom_motion_threshold` (`0.02` m): If the estimated init_pose change between consecutive ICP callbacks is less than this, TB3_2 is considered **stationary** (e.g., stuck behind an obstacle). ICP is skipped entirely in this state to prevent mismatched scans from polluting the map.
- `scan1_timeout_sec` (`1.0` s): If no `/TB3_1/scan` message is received for this duration (leader LiDAR lost), ICP is skipped and the last valid bridge TF is preserved.

**Protection flow summary:**
```
TB3_2 stuck behind obstacle
  → odom_motion_threshold not exceeded  → Skip ICP entirely          ✅
  → ICP runs but result jumps > 0.3 m   → Reject ICP result          ✅
  → TB3_1 scan disappears > 1.0 s       → Preserve last valid TF     ✅
```

---

## SLAM Toolbox Parameters (`param/slam_toolbox_params.yaml`)

The SLAM node runs only on the leader (`TB3_1`). Parameters are tuned conservatively to avoid map corruption caused by TF instability from the follower bridge.

| Parameter | Value | Rationale |
|---|---|---|
| `transform_timeout` | `0.5` s | Tolerates brief TF gaps without forcing map regeneration (was `0.2`). |
| `map_update_interval` | `3.0` s | Reduces map write frequency, limiting damage during unstable TF periods (was `2.0`). |
| `minimum_travel_distance` | `0.5` m | Only adds a new scan node after sufficient movement, reducing noise scans (was `0.3`). |
| `loop_match_maximum_variance_coarse` | `1.5` | ⭐ Tightens loop closure acceptance. The radial map explosion seen when TB3_2 gets stuck is caused by incorrect loop closures — this directly reduces that risk (was `3.0`). |

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
- `tf_timeout_sec` (`2.0` s): 장애물에 가려지거나 네트워크 끊김으로 인해 리더의 위치(TF)를 2.0초 이상 갱신받지 못하면, 팔로워는 즉시 현재 추종을 멈추고 **복구 모드(Recovery Mode)**로 진입합니다.
- `recovery_resend_period_sec` (`3.0` s): 복구 모드 지속 중 "마지막 알려진 위치로 이동" 목표를 **3초마다 재전송**합니다. Nav2가 목표를 완료 후 조용히 삭제하더라도 팔로워가 멈추지 않도록 보장합니다.

### Follower Detector 노드 파라미터 (`follower_detector_node.py`)

ICP 스캔 매칭으로 `TB3_1/odom`과 `TB3_2/odom` 좌표계를 실시간 동기화하는 노드의 파라미터입니다.

#### ICP 품질 및 혼합 비율

- `icp_fitness_threshold` (`0.3`): ICP 매칭 결과를 수용하기 위한 최소 품질 점수(0~1). 허위 매칭(False-positive)으로 위치가 불안정하면 이 값을 높이고, 특징이 적은 환경에서는 낮추세요. 기존 `0.15`에서 실환경 안정성을 위해 `0.3`으로 상향되었습니다.
- `blend_alpha` (`0.5`): 이전 TF 추정값과 새 ICP 결과를 혼합하는 비율. `0.0`이면 업데이트 없음(이전값 유지), `1.0`이면 ICP 결과를 그대로 사용합니다. 응답성을 높이려면 `0.7`로, 더 부드러운 필터링을 원하면 `0.3`으로 조정하세요.

#### 맵 오염 방지 (Map Corruption Protection)

TB3_2가 장애물에 막히거나 리더가 잠시 소실될 때 맵이 쉽게 오염되는 현상을 방지하는 파라미터입니다:

- `max_correction_dist` (`0.3` m): 새 ICP 결과가 현재 bridge TF와 이 거리 이상 차이 나면 **reject**합니다. 급격한 TF 점프가 SLAM 맵을 오염하는 것을 차단합니다.
- `max_correction_angle` (`0.5` rad ≈ 28°): ICP 결과의 각도 편이가 이 값 이상이면 reject합니다. bridge TF가 급자기 크게 회전하는 상황을 방지합니다.
- `odom_motion_threshold` (`0.02` m): ICP 콜백 간 init_pose 변화량이 이 값 이하면 TB3_2가 **정지**한 것으로 판단하고 ICP를 완전히 건너뜁니다. 장애물에 막힌 상태에서 두 로봇의 스캔 환경이 달라지면 ICP가 엉뚱한 결과를 낼 수 있기 때문입니다.
- `scan1_timeout_sec` (`1.0` s): `/TB3_1/scan` 메시지가 이 시간 이상 수신되지 않으면 (리더 LiDAR 소실) ICP를 건너뛰고 마지막 유효 bridge TF를 보존합니다.

**보호 동작 흐름:**
```
TB3_2 장애물에 막힌 상황
  → odom 변화량 < 0.02 m         → ICP 전체 건너뜀       ✅
  → ICP 결과가 0.3 m 이상 점프    → ICP 결과 reject       ✅
  → TB3_1 스캔 1.0s 이상 없음  → 마지막 TF 보존       ✅
```

### SLAM Toolbox 파라미터 (`param/slam_toolbox_params.yaml`)

SLAM은 리더 (`TB3_1`)에서만 실행됩니다. 팔로워 bridge TF 불안정에 의한 맵 오염을 최소화하도록 보수적으로 튜닝되었습니다.

| 파라미터 | 값 | 변경 이유 |
|---|---|---|
| `transform_timeout` | `0.5` s | 일시적 TF 끊김에도 SLAM이 맵을 재생성하지 않도록 내성 향상 (이전 `0.2`) |
| `map_update_interval` | `3.0` s | 맵 쓰기 빈도를 줄여 불안정한 TF 기간 중 손상을 제한 (이전 `2.0`) |
| `minimum_travel_distance` | `0.5` m | 충분한 이동 후에만 새 스캔 노드 추가, 노이즈 스캔 감소 (이전 `0.3`) |
| `loop_match_maximum_variance_coarse` | `1.5` | ⭐ 루프 클로저 허용 오차 축소. TB3_2가 막힌 상황에서 발생하는 방사형 맵 폭발을 방지하는 직접적 원인인 잘못된 루프 클로저를 방지 (이전 `3.0`) |

### Nav2 로컬 제어기 설정 (`escort_follower/param/controll_server1.yaml`)

팔로워 로봇의 `dwb_core::DWBLocalPlanner`는 공격적이고 빠른 자율주행보다는 부드러운 에스코트 주행을 위해 다음과 같이 특별하게 튜닝되었습니다:

- `min_vel_x: -0.12`: 리더가 급정거해서 간격이 너무 좁아졌을 때 팔로워가 천천히 뒤로 후진할 수 있게 해줍니다. 이 값이 `0.0`이면 로봇이 후진을 못 하고 제자리에서 180도 돌아버리는 기괴한 회피 기동을 하게 됩니다.
- `BaseObstacle.scale: 0.02`: 로컬 코스트맵의 장애물 회피 가중치를 의도적으로 아주 낮게 설정했습니다. 팔로워는 항상 리더 뒤에 바짝 붙어있어야 하므로, 장애물 회피 가중치가 너무 높으면 팔로워가 리더 자체를 '커다란 장애물'로 인식하고 다가가는 것을 무서워하게 되기 때문입니다.
- `PathDist.scale: 32.0`: 팔로워가 다른 어떤 요인보다도 생성된 목표점과 정확한 추종 경로(Path)를 밟고 따라가는 것을 최우선으로 여기도록 설정했습니다.
