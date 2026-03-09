// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Hyungyu Kim

#ifndef ESCORT_FOLLOWER__FOLLOWER_HPP_
#define ESCORT_FOLLOWER__FOLLOWER_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <chrono>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class Follower : public rclcpp::Node
{
public:
  explicit Follower(const std::string & node_name, const std::string & leader_name);

private:
  void tf_publisher();
  void send_path();
  bool get_target_pose();

  geometry_msgs::msg::TransformStamped leader_pose_in_tracking_frame_;
  geometry_msgs::msg::TransformStamped follower_pose_in_tracking_frame_;
  geometry_msgs::msg::PoseStamped prior_second_target_pose_;
  geometry_msgs::msg::PoseStamped last_sent_second_target_pose_;
  rclcpp::TimerBase::SharedPtr send_path_timer_;
  rclcpp::TimerBase::SharedPtr tf_publish_timer_;
  rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr nav2_action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr active_goal_handle_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string leader_name_;
  std::string follower_name_;
  bool use_sim_time_;
  bool publish_odom_bridge_;
  bool has_last_sent_goal_;
  bool has_prior_target_pose_;
  bool awaiting_goal_response_;
  bool applied_initial_step_;
  double follow_distance_;
  double initial_step_distance_;
  double goal_update_distance_threshold_;
  double goal_update_min_period_sec_;
  rclcpp::Time last_goal_sent_time_;
  std::string tracking_frame_;
};

#endif  // ESCORT_FOLLOWER__FOLLOWER_HPP_
