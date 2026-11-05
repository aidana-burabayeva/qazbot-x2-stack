#include <memory>

#include "qazbot_approach_governor/approach_governor_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;  // deterministic order, no callback races (Henki standard)
  // The executor holds only a weak reference: keep the node alive for the whole spin.
  auto node = std::make_shared<qazbot_approach_governor::ApproachGovernorNode>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
