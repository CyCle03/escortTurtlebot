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
