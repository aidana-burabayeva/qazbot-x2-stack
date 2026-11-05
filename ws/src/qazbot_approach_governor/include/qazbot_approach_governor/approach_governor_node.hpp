#pragma once

#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "qazbot_approach_governor/approach_governor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace qazbot_approach_governor
{

// Single responsibility: communication. All decisions are in ApproachGovernor / SpeedEstimator / PathProgress.
class ApproachGovernorNode : public rclcpp::Node
{
public:
  explicit ApproachGovernorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_plan(const nav_msgs::msg::Path::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_status();
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & params);
  double now_s() const;

  Params params_;
  bool shadow_{false};
  std::string map_frame_, base_frame_;
  ApproachGovernor governor_;
  SpeedEstimator speed_;
  PathProgress path_;
  Decision last_;
  uint64_t n_cmd_{0}, n_modified_{0}, n_brake_events_{0};
  State prev_state_{State::kPassThrough};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace qazbot_approach_governor
