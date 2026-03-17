# Parameters and Launch Configuration

This document explains the key launch arguments and YAML parameters used in the `escort_turtlebot_pkg` for configuring the simulation and Nav2 stacks.

## 1. Launch Arguments

Launch files in `launch/` dictate how the simulation starts and orchestrates the robots. The entry point is `escort_sim.launch.py`, which delegates to `escort_core.launch.py`.

### Key Arguments (`escort_sim.launch.py` / `escort_core.launch.py`)

-   **`use_sim_time`** (default: `true` / `false` dependent on launch file)
    Instructs ROS 2 nodes to utilize the simulation clock published by Gazebo `/clock` topic. Crucial for TF synchronization in simulation.
-   **`leader_x`**, **`leader_y`**, **`follower_x`**, **`follower_y`**
    The absolute starting coordinates (X, Y) for both robots in the Gazebo `map` frame.
-   **`odom_bridge_x`**, **`odom_bridge_y`**
    **(Only used if dynamic ICP matching is disabled)** Provides a static, hardcoded TF offset from `TB3_1/odom` to `TB3_2/odom`. The launch script calculates this dynamically using the spawn positions if the ICP node is not utilized.

## 2. Follower Detector Parameters (`follower_detector_node`)
These parameters control the ICP-based scan matching and coordinate synchronization.

- **`icp_fitness_threshold`** (default: `0.15`)
  Quality threshold for ICP matching. Lower values mean less strict matching.
- **`blend_alpha`** (default: `0.5`)
  Smoothing factor for TF updates (0.0 = keep old, 1.0 = use new).
- **`scan_timeout_sec`** (default: `1.0`)
  Seconds to wait before considering a LiDAR scan "stale".
- **`max_correction_dist`** (default: `0.3`)
  Maximum allowed jump (in meters) for a single ICP correction.
- **`odom_motion_threshold`** (default: `0.02`)
  Minimum odometry change required to trigger an ICP update (prevents drift when stationary).

## 3. Nav2 Configuration (`param/escort_controll_server1.yaml`)

This YAML file dictates the behavior of the `nav2_controller` (specifically the Follower's local planner/controller) and the `local_costmap`.

### DWBLocalPlanner (Controller)
The Follower utilizes the standard `dwb_core::DWBLocalPlanner` to navigate to the points behind the leader.

-   **Velocity Constraints (`min_vel_x`, `max_vel_x`)**
    -   `max_vel_x: 0.12`: Caps forward speed to a safe tracking pace.
    -   `min_vel_x: -0.12`: **Important!** Allows the follower to drive *backwards*. If the leader suddenly stops or the follower overshoots, a negative `min_vel_x` allows it to reverse to maintain the correct follow distance.
-   **`stateful: True`**
    The planner remembers previous trajectories, which helps prevent oscillation or jerky movements when re-planning rapidly moving targets.
-   **Critics (Cost Functions)**
    Critics determine how the controller evaluates generated trajectories.
    -   `PathAlign.scale` (12.0) / `GoalAlign.scale` (10.0): Highly weighted. Prioritizes keeping the robot strictly on the path and pointing towards the goal.
    -   `ObstacleFootprintCritic.scale` (1.0): Ensures the robot evaluates collision risks against the local costmap.

### Local Costmap Configuration
The Follower (`TB3_2`) does not maintain a global map. It only uses a local rolling window costmap to navigate and avoid immediate obstacles.

-   **`global_frame: TB3_2/odom`**
    The costmap is built relative to the follower's odometry frame, not the global SLAM map.
-   **`robot_base_frame: TB3_2/base_footprint`**
    The center of the costmap.
-   **`rolling_window: true`**
    Crucial for a local costmap; the map "moves" with the robot rather than remaining static.
-   **`width` / `height`** (`3` meters)
    The localized area (3x3 meters) around the robot where it checks for obstacles.
-   **Obstacle Layer (`nav2_costmap_2d::ObstacleLayer`)**
    -   Subscribes to `/TB3_2/scan`.
    -   `raytrace_range: 3.0` / `obstacle_range: 2.5`: Detects obstacles up to 2.5m away and clears empty space up to 3.0m away using the LiDAR.

---

## 한국어 설명 (Korean Explanation)

# 파라미터 및 런치(Launch) 설정

이 문서는 시뮬레이션 및 Nav2 스택을 구성하기 위해 `escort_turtlebot_pkg`에서 사용되는 주요 런치 인자와 YAML 파라미터에 대해 설명합니다.

## 1. 런치 인자 (Launch Arguments)

`launch/` 디렉터리의 런치 파일들은 시뮬레이션을 시작하고 로봇들을 제어하는 방식을 정의합니다. 진입점은 `escort_sim.launch.py`이며, 이는 내부적으로 `escort_core.launch.py`를 호출합니다.

### 주요 인자 (`escort_sim.launch.py` / `escort_core.launch.py`)

-   **`use_sim_time`** (기본값: 런치 파일에 따라 `true` 또는 `false`)
    ROS 2 노드들이 Gazebo의 `/clock` 토픽에서 발행하는 시뮬레이션 시간을 사용하도록 지시합니다. 시뮬레이션 환경 내의 TF 동기화를 위해 필수적인 설정입니다.
-   **`leader_x`**, **`leader_y`**, **`follower_x`**, **`follower_y`**
    Gazebo `map` 프레임 내에서 두 로봇이 스폰될 절대적 시작 좌표(X, Y)입니다.
-   **`odom_bridge_x`**, **`odom_bridge_y`**
    **(동적 ICP 매칭이 비활성화된 경우에만 사용)** `TB3_1/odom`에서 `TB3_2/odom`로의 정적(고정된) TF 오프셋을 제공합니다. ICP 노드를 사용하지 않는 경우, 런치 스크립트가 로봇의 스폰 좌표를 바탕으로 이 값을 자동으로 계산합니다.

## 2. Follower Detector 파라미터 (`follower_detector_node`)
ICP 기반 스캔 매칭 및 좌표계 동기화를 제어하는 파라미터입니다.

- **`icp_fitness_threshold`** (기본값: `0.15`)
  ICP 매칭 품질 임계값. 이 값보다 매칭 품질이 높아야 TF 업데이트가 수행됩니다.
- **`blend_alpha`** (기본값: `0.5`)
  TF 업데이트 부드러움 조절 계수 (0에 가까울수록 이전 값 유지, 1에 가까울수록 새 결과 반영).
- **`scan_timeout_sec`** (기본값: `1.0`)
  라이다 스캔 데이터가 유효하다고 판단하는 최대 시간(초).
- **`max_correction_dist`** (기본값: `0.3`)
  단일 ICP 보정으로 허용되는 최대 이동 거리(m). 급격한 위치 튀김을 방지합니다.
- **`odom_motion_threshold`** (기본값: `0.02`)
  ICP 업데이트를 트리거하기 위한 최소 오도메트리 변화량(m). 정지 상태에서의 맵 오염을 방지합니다.

## 3. Nav2 설정 (`param/escort_controll_server1.yaml`)

이 YAML 파일은 `nav2_controller` (특히 팔로워의 로컬 플래너/컨트롤러)와 `local_costmap`의 작동 방식을 정의합니다.

### DWBLocalPlanner (컨트롤러)
팔로워 로봇은 리더 뒷편의 타겟 지점으로 이동하기 위해 표준 `dwb_core::DWBLocalPlanner`를 사용합니다.

-   **속도 제한 (`min_vel_x`, `max_vel_x`)**
    -   `max_vel_x: 0.12`: 전진 속도를 안정적인 수준으로 제한합니다.
    -   `min_vel_x: -0.12`: **중요!** 팔로워가 *후진*할 수 있도록 허용합니다. 리더가 갑자기 멈추거나 팔로워가 타겟을 지나친 경우, 음수의 `min_vel_x` 설정을 통해 로봇이 뒤로 이동하며 올바른 추종 간격을 유지할 수 있습니다.
-   **`stateful: True`**
    플래너가 이전 궤적을 기억하게 합니다. 목적지가 계속해서 변하는 상황에서 진행 방향이 급격하게 흔들리거나 요동치는 현상(oscillation)을 방지합니다.
-   **코스트 함수 / Critics (평가 지표)**
    생성된 궤적을 컨트롤러가 얼마나 좋은 경로로 평가할지 결정합니다.
    -   `PathAlign.scale` (12.0) / `GoalAlign.scale` (10.0): 높은 가중치가 부여되었습니다. 로봇이 경로를 정확히 따르고 목표점을 바라보도록 우선합니다.
    -   `ObstacleFootprintCritic.scale` (1.0): 로컬 코스트맵 상 충돌 위험도를 평가에 반영합니다.

### 로컬 코스트맵 설정 (Local Costmap Configuration)
팔로워 로봇(`TB3_2`)은 별도의 글로벌 맵을 유지하지 않습니다. 즉각적인 장애물을 인지하고 회피하기 위해 이동하는 로컬 윈도우(Rolling Window) 코스트맵만을 사용합니다.

-   **`global_frame: TB3_2/odom`**
    코스트맵이 글로벌 SLAM 맵이 아닌, 팔로워 로봇의 자체 오도메트리 기반(odom)으로 생성됩니다.
-   **`robot_base_frame: TB3_2/base_footprint`**
    코스트맵의 정중앙 기준점입니다.
-   **`rolling_window: true`**
    로컬 코스트맵의 필수 설정으로, 맵이 로봇의 이동에 따라 고정되지 않고 함께 이동하도록 합니다.
-   **`width` / `height`** (`3` 미터)
    로봇이 장애물을 탐색할 주변의 국소 반경(3x3 미터 영역)입니다.
-   **장애물 레이어 (`nav2_costmap_2d::ObstacleLayer`)**
    -   `/TB3_2/scan` 데이터를 구독합니다.
    -   `raytrace_range: 3.0` / `obstacle_range: 2.5`: 반경 2.5m 내의 장애물을 맵에 표기하고, LiDAR를 이용해 최대 3.0m까지의 빈 공간을 비워냅니다.
