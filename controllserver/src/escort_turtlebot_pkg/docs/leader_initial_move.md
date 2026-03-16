# Leader Initial Move Node

The `leader_initial_move_node` is a utility node that provides a simple, one-time movement for a robot at startup. Its primary purpose is to create an initial separation between the leader and follower robots, which can be useful for scenarios where the robots start very close to each other.

## How It Works
The node waits for a specified startup delay, then publishes a `geometry_msgs/Twist` message to a specified `cmd_vel` topic to move the robot forward or backward. It monitors the robot's odometry from a specified `odom` topic to track the distance traveled. The movement stops when either the target distance is reached or a maximum duration has elapsed.

## Key Parameters
- `cmd_vel_topic` (string, default: `/TB3_1/cmd_vel`): The topic to publish `Twist` commands to.
- `odom_topic` (string, default: `/TB3_1/odom`): The topic to subscribe to for odometry information.
- `distance` (double, default: `0.5`): The target distance to move, in meters. A positive value moves the robot forward, and a negative value moves it backward.
- `speed` (double, default: `0.10`): The linear speed of the robot, in meters per second.
- `startup_delay_sec` (double, default: `2.0`): The delay in seconds before the movement starts.
- `max_duration_sec` (double, default: `10.0`): The maximum duration in seconds for the movement. The robot will stop even if it hasn't reached the target distance.

## Standalone Execution
You can run this node standalone for testing or to perform a simple movement:
```bash
ros2 run escort_turtlebot_pkg leader_initial_move_node --ros-args -p distance:=0.3
```

---

## 한국어 안내

`leader_initial_move_node`는 시작 시 로봇을 한 번 움직여주는 유틸리티 노드입니다. 이 노드의 주된 목적은 리더와 팔로워 로봇 사이에 초기 거리를 만들어주는 것으로, 로봇들이 서로 매우 가깝게 시작하는 시나리오에서 유용합니다.

## 동작 방식
노드는 지정된 시작 지연 시간 동안 기다린 다음, 지정된 `cmd_vel` 토픽으로 `geometry_msgs/Twist` 메시지를 발행하여 로봇을 앞뒤로 움직입니다. 지정된 `odom` 토픽에서 로봇의 주행 거리를 모니터링하여 이동 거리를 추적합니다. 목표 거리에 도달하거나 최대 지속 시간이 지나면 움직임이 멈춥니다.

## 주요 파라미터
- `cmd_vel_topic` (string, 기본값: `/TB3_1/cmd_vel`): `Twist` 명령을 발행할 토픽.
- `odom_topic` (string, 기본값: `/TB3_1/odom`): 주행 거리 정보를 구독할 토픽.
- `distance` (double, 기본값: `0.5`): 목표 이동 거리 (미터). 양수 값은 로봇을 앞으로, 음수 값은 뒤로 움직입니다.
- `speed` (double, 기본값: `0.10`): 로봇의 선형 속도 (m/s).
- `startup_delay_sec` (double, 기본값: `2.0`): 움직임이 시작되기 전의 지연 시간 (초).
- `max_duration_sec` (double, 기본값: `10.0`): 움직임의 최대 지속 시간 (초). 목표 거리에 도달하지 못했더라도 로봇이 멈춥니다.

## 단독 실행
테스트나 간단한 움직임을 수행하기 위해 이 노드를 단독으로 실행할 수 있습니다:
```bash
ros2 run escort_turtlebot_pkg leader_initial_move_node --ros-args -p distance:=0.3
```
