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

#include "escort_follower/follower.hpp"

#include <algorithm>
#include <cmath>


Follower::Follower(const std::string & follower_name, const std::string & leader_name)
: Node(follower_name + "_follower_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  leader_name_(leader_name),
  follower_name_(follower_name),
  has_last_sent_goal_(false),
  has_prior_target_pose_(false),
  awaiting_goal_response_(false),
  last_goal_sent_time_(0, 0, this->get_clock()->get_clock_type())
{
  this->declare_parameter<bool>("publish_odom_bridge", true);
  this->declare_parameter<double>("follow_distance", 0.5);
  this->declare_parameter<double>("goal_update_distance_threshold", 0.03);
  this->declare_parameter<double>("goal_update_min_period_sec", 0.3);
  this->declare_parameter<std::string>("tracking_frame", "map");
  if (!get_parameter("use_sim_time", use_sim_time_)) {
    use_sim_time_ = false;
  }
  get_parameter("publish_odom_bridge", publish_odom_bridge_);
  get_parameter("follow_distance", follow_distance_);
  get_parameter("goal_update_distance_threshold", goal_update_distance_threshold_);
  get_parameter("goal_update_min_period_sec", goal_update_min_period_sec_);
  get_parameter("tracking_frame", tracking_frame_);

  prior_second_target_pose_.pose.orientation.w = 1.0;
  last_sent_second_target_pose_.pose.orientation.w = 1.0;

  if (publish_odom_bridge_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Follower::tf_publisher, this));
  }
  nav2_action_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(
    this,
    follower_name_ + "/follow_path");

  send_path_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&Follower::send_path, this));

  RCLCPP_INFO(
    this->get_logger(),
    "[%s] initialized successfully",
    this->get_name());
}

void Follower::tf_publisher()
{
  if (!publish_odom_bridge_ || !tf_broadcaster_) {
    return;
  }
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = this->get_clock()->now();
  tf_msg.header.frame_id = this->leader_name_ + "/odom";
  tf_msg.child_frame_id = this->follower_name_ + "/odom";

  if (this->use_sim_time_ == true) {
    tf_msg.transform.translation.x = 0;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
  } else {
    tf_msg.transform.translation.x = -0.14;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
  }
  this->tf_broadcaster_->sendTransform(tf_msg);
}

bool Follower::get_target_pose()
{
  try {
    this->leader_pose_in_tracking_frame_ = this->tf_buffer_.lookupTransform(
      tracking_frame_,
      this->leader_name_ + "/base_footprint",
      tf2::TimePointZero);
    this->follower_pose_in_tracking_frame_ = this->tf_buffer_.lookupTransform(
      tracking_frame_,
      this->follower_name_ + "/base_footprint",
      tf2::TimePointZero);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Waiting for TF in tracking frame '%s'", tracking_frame_.c_str());
    return false;
  }
}

void Follower::send_path()
{
  if (!get_target_pose()) {
    return;
  }

  if (!this->nav2_action_client_->wait_for_action_server(std::chrono::seconds(0))) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Action server not available");
    return;
  }

  if (awaiting_goal_response_) {
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = tracking_frame_;

  geometry_msgs::msg::PoseStamped first_target_pose;
  first_target_pose.header.stamp = this->get_clock()->now();
  first_target_pose.header.frame_id = tracking_frame_;
  if (has_prior_target_pose_) {
    first_target_pose.pose = this->prior_second_target_pose_.pose;
  } else {
    first_target_pose.pose.position.x = this->follower_pose_in_tracking_frame_.transform.translation.x;
    first_target_pose.pose.position.y = this->follower_pose_in_tracking_frame_.transform.translation.y;
    first_target_pose.pose.orientation = this->follower_pose_in_tracking_frame_.transform.rotation;
  }
  path.poses.push_back(first_target_pose);

  geometry_msgs::msg::PoseStamped second_target_pose;
  second_target_pose.header.stamp = this->get_clock()->now();
  second_target_pose.header.frame_id = tracking_frame_;
  const double leader_x = this->leader_pose_in_tracking_frame_.transform.translation.x;
  const double leader_y = this->leader_pose_in_tracking_frame_.transform.translation.y;
  const double follower_x = this->follower_pose_in_tracking_frame_.transform.translation.x;
  const double follower_y = this->follower_pose_in_tracking_frame_.transform.translation.y;
  const double dx = leader_x - follower_x;
  const double dy = leader_y - follower_y;
  const double distance = std::hypot(dx, dy);
  if (distance > 1e-6) {
    const double scale = std::max(0.0, (distance - this->follow_distance_) / distance);
    second_target_pose.pose.position.x = follower_x + dx * scale;
    second_target_pose.pose.position.y = follower_y + dy * scale;
    const double yaw = std::atan2(dy, dx);
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    second_target_pose.pose.orientation = tf2::toMsg(quat);
  } else {
    second_target_pose.pose.position.x = follower_x;
    second_target_pose.pose.position.y = follower_y;
    second_target_pose.pose.orientation = this->follower_pose_in_tracking_frame_.transform.rotation;
  }
  path.poses.push_back(second_target_pose);
  this->prior_second_target_pose_ = second_target_pose;
  has_prior_target_pose_ = true;

  const rclcpp::Time now = this->get_clock()->now();
  if (has_last_sent_goal_) {
    const double delta_x =
      second_target_pose.pose.position.x - last_sent_second_target_pose_.pose.position.x;
    const double delta_y =
      second_target_pose.pose.position.y - last_sent_second_target_pose_.pose.position.y;
    const double delta_distance = std::hypot(delta_x, delta_y);
    const bool is_small_change = delta_distance < goal_update_distance_threshold_;
    const bool is_too_soon =
      (now - last_goal_sent_time_).seconds() < goal_update_min_period_sec_;
    if (is_small_change || is_too_soon) {
      return;
    }
  }

  auto goal_msg = nav2_msgs::action::FollowPath::Goal();
  goal_msg.path = path;

  rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions goal_options;
  goal_options.goal_response_callback =
    [this](rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr goal_handle) {
      awaiting_goal_response_ = false;
      active_goal_handle_ = goal_handle;
      if (!goal_handle) {
        RCLCPP_WARN(this->get_logger(), "FollowPath goal rejected");
      }
    };
  goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult &) {
      active_goal_handle_.reset();
    };

  if (active_goal_handle_) {
    this->nav2_action_client_->async_cancel_goal(active_goal_handle_);
    active_goal_handle_.reset();
  }

  awaiting_goal_response_ = true;
  last_goal_sent_time_ = now;
  last_sent_second_target_pose_ = second_target_pose;
  has_last_sent_goal_ = true;
  this->nav2_action_client_->async_send_goal(goal_msg, goal_options);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int number = 0;
  if (argc > 1) {
    try {
      number = std::stoi(argv[1]);
    } catch (const std::exception & e) {
      std::cerr << "Invalid number of followers: " << argv[1] << std::endl;
      return 1;
    }
  }

  std::vector<std::shared_ptr<Follower>> followers;
  for (int i = 1; i <= number; ++i) {
    auto node = std::make_shared<Follower>(
      "TB3_" + std::to_string(i + 1),
      "TB3_" + std::to_string(i));
    followers.push_back(node);
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  for (auto & node : followers) {
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
