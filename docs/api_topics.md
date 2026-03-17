# ROS 2 API Reference: Topics & TFs

This document provides a centralized reference for the ROS 2 topics, actions, and TF frames used across the `escort_turtlebot` project.

## 1. Important TF Frames

| Frame ID | Description | Root/Parent |
| :--- | :--- | :--- |
| `map` | Global map coordinate system. | N/A (Static) |
| `TB3_1/odom` | Leader's odometry frame. | `map` |
| `TB3_1/base_footprint` | Leader's robot center. | `TB3_1/odom` |
| `TB3_2/odom` | Follower's odometry frame. | `TB3_1/odom` (Dynamic Bridge) |
| `TB3_2/base_footprint` | Follower's robot center. | `TB3_2/odom` |

## 2. Topic Registry

### Follower Control (`escort_follower`)
- **`/ultrasonic_distance`** (`std_msgs/Float32`)
  - Distance to nearest obstacle in front (cm). Triggers emergency stop.
- **`/{follower_name}/cmd_vel`** (`geometry_msgs/Twist`)
  - Velocity commands for the follower robot.

### Perception & Sync (`escort_turtlebot_pkg`)
- **`/TB3_1/scan`**, **`/TB3_2/scan`** (`sensor_msgs/LaserScan`)
  - LiDAR data used for ICP-based TF alignment and local costmaps.
- **`/TB3_1/cmd_vel`** (`geometry_msgs/Twist`)
  - Velocity commands for the leader robot.

## 3. Actions

- **`/{follower_name}/follow_path`** (`nav2_msgs/action/FollowPath`)
  - Navigation action server used by the follower to track virtual targets.

---

# ROS 2 API 참조 (토픽 및 TF)

이 문서는 `escort_turtlebot` 프로젝트에서 사용되는 ROS 2 토픽, 액션, 그리고 TF 프레임에 대한 통합 참조 정보를 제공합니다.

## 1. 주요 TF 프레임

| 프레임 ID | 설명 | 부모 프레임 |
| :--- | :--- | :--- |
| `map` | 글로벌 지도 좌표계. | 없음 (최상위) |
| `TB3_1/odom` | 리더 로봇의 오도메트리 프레임. | `map` |
| `TB3_1/base_footprint` | 리더 로봇의 중심점. | `TB3_1/odom` |
| `TB3_2/odom` | 팔로워 로봇의 오도메트리 프레임. | `TB3_1/odom` (동적 브리지) |
| `TB3_2/base_footprint` | 팔로워 로봇의 중심점. | `TB3_2/odom` |

## 2. 토픽 레지스트리

### 팔로워 제어 (`escort_follower`)
- **`/ultrasonic_distance`** (`std_msgs/Float32`)
  - 전방 장애물까지의 거리 (cm). 10cm 이내일 경우 긴급 후진을 트리거합니다.
- **`/{follower_name}/cmd_vel`** (`geometry_msgs/Twist`)
  - 팔로워 로봇의 실제 속도 제어 명령.

### 인식 및 동기화 (`escort_turtlebot_pkg`)
- **`/TB3_1/scan`**, **`/TB3_2/scan`** (`sensor_msgs/LaserScan`)
  - 라이다 데이터. ICP 기반 TF 정합 및 로컬 코스트맵 생성에 사용됩니다.
- **`/TB3_1/cmd_vel`** (`geometry_msgs/Twist`)
  - 리더 로봇의 속도 제어 명령 (조종기 입력 등).

## 3. 액션 (Actions)

- **`/{follower_name}/follow_path`** (`nav2_msgs/action/FollowPath`)
  - 팔로워가 가상 타겟 경로를 따라가기 위해 사용하는 Nav2 액션 서버입니다.
