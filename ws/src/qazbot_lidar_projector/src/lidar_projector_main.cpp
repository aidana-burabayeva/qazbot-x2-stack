// Copyright 2026 Qazbot
#include <memory>

#include "qazbot_lidar_projector/lidar_projector_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<qazbot_lidar_projector::LidarProjectorNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
