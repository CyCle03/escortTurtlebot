#include <gtest/gtest.h>
#include "escort_follower/follower.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class FollowerCoreTest : public ::testing::Test {
protected:
  void SetUp() override {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
  }

  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

TEST_F(FollowerCoreTest, GetTargetPose_Success) {
  // Arrange
  auto follower_core = FollowerCore("follower", "leader", 0.5, tf_buffer_);
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = clock_->now();
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = "leader/base_footprint";
  transform_stamped.transform.translation.x = 1.0;
  transform_stamped.transform.translation.y = 2.0;
  transform_stamped.transform.rotation.w = 1.0;
  tf_buffer_->setTransform(transform_stamped, "test", false);

  // Act
  geometry_msgs::msg::TransformStamped leader_pose;
  bool result = follower_core.get_target_pose(leader_pose, "map");

  // Assert
  ASSERT_TRUE(result);
  EXPECT_EQ(leader_pose.transform.translation.x, 1.0);
  EXPECT_EQ(leader_pose.transform.translation.y, 2.0);
}

TEST_F(FollowerCoreTest, GetTargetPose_Fail) {
  // Arrange
  auto follower_core = FollowerCore("follower", "leader", 0.5, tf_buffer_);

  // Act
  geometry_msgs::msg::TransformStamped leader_pose;
  bool result = follower_core.get_target_pose(leader_pose, "map");

  // Assert
  ASSERT_FALSE(result);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
