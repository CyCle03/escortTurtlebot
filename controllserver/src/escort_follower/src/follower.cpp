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

#include <tf2/utils.h>


Follower::Follower(const std::string & follower_name, const std::string & leader_name)
: Node(follower_name + "_follower_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  leader_name_(leader_name),
  follower_name_(follower_name),
  has_last_sent_goal_(false),
  has_prior_target_pose_(false),
  awaiting_goal_response_(false),
  applied_initial_step_(false),
  has_last_known_follower_pose_(false),
  last_goal_sent_time_(0, 0, this->get_clock()->get_clock_type()),
  last_tf_success_time_(0, 0, this->get_clock()->get_clock_type()),
  last_recovery_goal_sent_time_(0, 0, this->get_clock()->get_clock_type()),
  is_in_recovery_mode_(false)
{
  this->declare_parameter<bool>("publish_odom_bridge", true);
  this->declare_parameter<double>("follow_distance", 0.5);
  this->declare_parameter<double>("initial_step_distance", 0.0);
  this->declare_parameter<double>("goal_update_distance_threshold", 0.03);
  this->declare_parameter<double>("goal_update_min_period_sec", 0.3);
  this->declare_parameter<double>("tf_timeout_sec", 2.0);
  this->declare_parameter<double>("recovery_resend_period_sec", 3.0);
  this->declare_parameter<std::string>("tracking_frame", "map");
  if (!get_parameter("use_sim_time", use_sim_time_)) {
    use_sim_time_ = false;
  }
  get_parameter("publish_odom_bridge", publish_odom_bridge_);
  get_parameter("follow_distance", follow_distance_);
  get_parameter("initial_step_distance", initial_step_distance_);
  get_parameter("goal_update_distance_threshold", goal_update_distance_threshold_);
  get_parameter("goal_update_min_period_sec", goal_update_min_period_sec_);
  get_parameter("tf_timeout_sec", tf_timeout_sec_);
  get_parameter("recovery_resend_period_sec", recovery_resend_period_sec_);
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
  bool leader_ok = false;
  try {
    this->leader_pose_in_tracking_frame_ = this->tf_buffer_.lookupTransform(
      tracking_frame_,
      this->leader_name_ + "/base_footprint",
      tf2::TimePointZero);
    
    this->last_known_leader_pose_ = this->leader_pose_in_tracking_frame_;
    this->last_tf_success_time_ = this->get_clock()->now();
    if (this->is_in_recovery_mode_) {
      RCLCPP_INFO(this->get_logger(), "Leader found! Exiting recovery mode.");
      this->is_in_recovery_mode_ = false;
    }
    leader_ok = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Waiting for leader TF in tracking frame '%s'", tracking_frame_.c_str());
  }

  try {
    this->follower_pose_in_tracking_frame_ = this->tf_buffer_.lookupTransform(
      tracking_frame_,
      this->follower_name_ + "/base_footprint",
      tf2::TimePointZero);
    // 팩로워 위치를 캐시에 저장 (Recovery 시 팩로워 TF 다운 때 사용)
    this->last_known_follower_pose_ = this->follower_pose_in_tracking_frame_;
    this->has_last_known_follower_pose_ = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Waiting for follower TF in tracking frame '%s'", tracking_frame_.c_str());
    if (!has_last_known_follower_pose_) {
      // 캐시도 없으면 스케줄링 불가
      return false;
    }
    // 캐시된 팩로워 위치를 사용하여 Recovery가 진행될 수 있도록 허용
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Follower TF unavailable, using cached pose for recovery.");
    this->follower_pose_in_tracking_frame_ = this->last_known_follower_pose_;
  }

  return leader_ok;
}

void Follower::send_path()
{
  bool leader_visible = get_target_pose();
  bool trigger_recovery_now = false;

  if (!leader_visible) {
    double time_since_last_success = (this->get_clock()->now() - last_tf_success_time_).seconds();
    if (time_since_last_success > tf_timeout_sec_ && !is_in_recovery_mode_ && last_tf_success_time_.nanoseconds() > 0) {
      RCLCPP_WARN(this->get_logger(), "Leader lost for %.1f sec! Entering Recovery Mode.", time_since_last_success);
      is_in_recovery_mode_ = true;
      trigger_recovery_now = true;
    } else if (is_in_recovery_mode_) {
      // Recovery Mode 지속 중: 주기적으로 마지막 알려진 위치로 목표 재전송
      double time_since_recovery_goal =
        (this->get_clock()->now() - last_recovery_goal_sent_time_).seconds();
      if (last_recovery_goal_sent_time_.nanoseconds() == 0 ||
          time_since_recovery_goal > recovery_resend_period_sec_)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Still in recovery mode. Re-sending goal to last known position (%.1f sec elapsed).",
          time_since_last_success);
        trigger_recovery_now = true;
      } else {
        return;
      }
    } else {
      return;
    }
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

  const double follower_x = this->follower_pose_in_tracking_frame_.transform.translation.x;
  const double follower_y = this->follower_pose_in_tracking_frame_.transform.translation.y;

  if (trigger_recovery_now) {
    // Recovery: 리더 마지막 확인 위치로 이동
    second_target_pose.pose.position.x = this->last_known_leader_pose_.transform.translation.x;
    second_target_pose.pose.position.y = this->last_known_leader_pose_.transform.translation.y;
    second_target_pose.pose.orientation = this->last_known_leader_pose_.transform.rotation;
    last_recovery_goal_sent_time_ = this->get_clock()->now();
    // Recovery 첫 번째 waypoint: 이전 목표점 또는 팩로워 마지막 알려진 위치
    if (!has_prior_target_pose_ && has_last_known_follower_pose_) {
      first_target_pose.pose.position.x = this->last_known_follower_pose_.transform.translation.x;
      first_target_pose.pose.position.y = this->last_known_follower_pose_.transform.translation.y;
      first_target_pose.pose.orientation = this->last_known_follower_pose_.transform.rotation;
    }
  } else {
    // Normal hybrid target generation
    const double leader_x = this->leader_pose_in_tracking_frame_.transform.translation.x;
    const double leader_y = this->leader_pose_in_tracking_frame_.transform.translation.y;
    const auto & leader_q_msg = this->leader_pose_in_tracking_frame_.transform.rotation;
    tf2::Quaternion leader_q;
    tf2::fromMsg(leader_q_msg, leader_q);
    const double leader_yaw = tf2::getYaw(leader_q);

    const double target_x = leader_x - this->follow_distance_ * std::cos(leader_yaw);
    const double target_y = leader_y - this->follow_distance_ * std::sin(leader_yaw);

    const double dx = target_x - follower_x;
    const double dy = target_y - follower_y;
    const double target_distance = std::hypot(dx, dy);
    
    if (target_distance > 1e-6) {
      if (!applied_initial_step_ && initial_step_distance_ > 0.0) {
        const double step = std::min(initial_step_distance_, target_distance);
        second_target_pose.pose.position.x = follower_x + (dx / target_distance) * step;
        second_target_pose.pose.position.y = follower_y + (dy / target_distance) * step;
        applied_initial_step_ = true;
      } else {
        second_target_pose.pose.position.x = target_x;
        second_target_pose.pose.position.y = target_y;
      }
      tf2::Quaternion quat;
      quat.setRPY(0.0, 0.0, std::atan2(dy, dx));
      second_target_pose.pose.orientation = tf2::toMsg(quat);
    } else {
      second_target_pose.pose.position.x = follower_x;
      second_target_pose.pose.position.y = follower_y;
      second_target_pose.pose.orientation = this->follower_pose_in_tracking_frame_.transform.rotation;
    }
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
