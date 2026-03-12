#include "escort_follower/follower.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <follower_name> <leader_name>" << std::endl;
    return 1;
  }

  std::string follower_name = argv[1];
  std::string leader_name = argv[2];

  auto follower_node = std::make_shared<Follower>(follower_name, leader_name);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(follower_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
