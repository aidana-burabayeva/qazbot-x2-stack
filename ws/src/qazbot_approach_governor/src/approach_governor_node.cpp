#include "qazbot_approach_governor/approach_governor_node.hpp"

#include <algorithm>
#include <chrono>

#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/exceptions.h"

namespace qazbot_approach_governor
{

namespace
{
const char * state_name(State s)
{
  switch (s) {
    case State::kPassThrough: return "pass_through";
    case State::kBraking: return "braking";
    case State::kHeldAtStop: return "held_at_stop";
    case State::kNoData: return "no_data";
  }
  return "?";
}

diagnostic_msgs::msg::KeyValue kv(const std::string & k, const std::string & v)
{
  diagnostic_msgs::msg::KeyValue m;
  m.key = k;
  m.value = v;
  return m;
}
}  // namespace

ApproachGovernorNode::ApproachGovernorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("approach_governor", options)
{
  params_.stop_time_s = declare_parameter("stop_time_s", params_.stop_time_s);
  params_.stop_margin_m = declare_parameter("stop_margin_m", params_.stop_margin_m);
  params_.speed_window_s = declare_parameter("speed_window_s", params_.speed_window_s);
  params_.max_speed_age_s = declare_parameter("max_speed_age_s", params_.max_speed_age_s);
  params_.max_plan_age_s = declare_parameter("max_plan_age_s", params_.max_plan_age_s);
  params_.max_pose_age_s = declare_parameter("max_pose_age_s", params_.max_pose_age_s);
  params_.stopped_speed_mps = declare_parameter("stopped_speed_mps", params_.stopped_speed_mps);
  params_.max_brake_hold_s = declare_parameter("max_brake_hold_s", params_.max_brake_hold_s);
  params_.min_forward_cmd = declare_parameter("min_forward_cmd", params_.min_forward_cmd);
  shadow_ = declare_parameter("shadow", false);  // true: observe and report, never publish a command
  map_frame_ = declare_parameter("plan_frame", std::string("map"));
  base_frame_ = declare_parameter("base_frame", std::string("base_footprint"));
  const double status_period = declare_parameter("status_period_s", 0.5);

  const std::string bad = params_.validate();
  if (!bad.empty()) {
    RCLCPP_FATAL(get_logger(), "invalid parameters: %s", bad.c_str());
    throw std::invalid_argument(bad);
  }
  governor_.set_params(params_);
  speed_.configure(params_.speed_window_s, params_.max_speed_age_s);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_in", rclcpp::QoS(10), std::bind(&ApproachGovernorNode::on_cmd, this, std::placeholders::_1));
  plan_sub_ = create_subscription<nav_msgs::msg::Path>(
    "plan", rclcpp::QoS(10), std::bind(&ApproachGovernorNode::on_plan, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(), std::bind(&ApproachGovernorNode::on_odom, this, std::placeholders::_1));
  if (!shadow_) {cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", rclcpp::QoS(10));}
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("status", rclcpp::QoS(10));
  status_timer_ = create_wall_timer(
    std::chrono::duration<double>(std::max(0.1, status_period)), std::bind(&ApproachGovernorNode::publish_status, this));
  param_cb_ = add_on_set_parameters_callback(
    std::bind(&ApproachGovernorNode::on_parameters, this, std::placeholders::_1));
  RCLCPP_INFO(
    get_logger(), "approach_governor started (shadow=%s, stop_time_s=%.2f)", shadow_ ? "true" : "false",
    params_.stop_time_s);
}

double ApproachGovernorNode::now_s() const {return now().seconds();}

void ApproachGovernorNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & p = msg->pose.pose.position;
  speed_.push(rclcpp::Time(msg->header.stamp).seconds(), p.x, p.y);
}

void ApproachGovernorNode::on_plan(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->header.frame_id != map_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "plan frame '%s' != '%s': plan ignored", msg->header.frame_id.c_str(),
      map_frame_.c_str());
    path_.clear();
    return;
  }
  std::vector<Point2> pts;
  pts.reserve(msg->poses.size());
  for (const auto & ps : msg->poses) {pts.push_back({ps.pose.position.x, ps.pose.position.y});}
  path_.set_path(pts, now_s());
}

void ApproachGovernorNode::on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double t = now_s();
  std::optional<double> remaining;
  if (path_.has_path() && t - path_.received() <= params_.max_plan_age_s) {
    try {
      const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      const double age = t - rclcpp::Time(tf.header.stamp).seconds();
      if (age <= params_.max_pose_age_s) {
        remaining = path_.remaining({tf.transform.translation.x, tf.transform.translation.y});
      }
    } catch (const tf2::TransformException &) {
      // no pose: remaining stays unknown, command passes (or an existing latch keeps braking)
    }
  }
  Command c{msg->linear.x, msg->linear.y, msg->angular.z};
  last_ = governor_.update(t, c, speed_.speed(t), remaining);
  ++n_cmd_;
  if (last_.modified) {++n_modified_;}
  if (last_.state == State::kBraking && prev_state_ != State::kBraking) {
    ++n_brake_events_;
    RCLCPP_INFO(
      get_logger(), "brake: speed %.2f m/s, remaining %.2f m <= stop distance %.2f m", last_.speed, last_.remaining,
      last_.stop_dist);
  }
  prev_state_ = last_.state;
  if (cmd_pub_) {
    geometry_msgs::msg::Twist out = *msg;  // all other fields (vy, wz, ...) forwarded untouched
    out.linear.x = last_.out.vx;
    cmd_pub_->publish(out);
  }
}

void ApproachGovernorNode::publish_status()
{
  diagnostic_msgs::msg::DiagnosticStatus s;
  s.name = "approach_governor";
  s.hardware_id = "qazbot";
  s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  s.message = state_name(last_.state);
  s.values = {
    kv("shadow", shadow_ ? "true" : "false"),   kv("speed_mps", std::to_string(last_.speed)),
    kv("remaining_m", std::to_string(last_.remaining)), kv("stop_dist_m", std::to_string(last_.stop_dist)),
    kv("commands", std::to_string(n_cmd_)),      kv("modified", std::to_string(n_modified_)),
    kv("brake_events", std::to_string(n_brake_events_))};
  status_pub_->publish(s);
}

rcl_interfaces::msg::SetParametersResult ApproachGovernorNode::on_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  Params next = params_;
  for (const auto & p : params) {
    const auto & n = p.get_name();
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {continue;}
    const double v = p.as_double();
    if (n == "stop_time_s") {next.stop_time_s = v;}
    else if (n == "stop_margin_m") {next.stop_margin_m = v;}
    else if (n == "speed_window_s") {next.speed_window_s = v;}
    else if (n == "max_speed_age_s") {next.max_speed_age_s = v;}
    else if (n == "max_plan_age_s") {next.max_plan_age_s = v;}
    else if (n == "max_pose_age_s") {next.max_pose_age_s = v;}
    else if (n == "stopped_speed_mps") {next.stopped_speed_mps = v;}
    else if (n == "max_brake_hold_s") {next.max_brake_hold_s = v;}
    else if (n == "min_forward_cmd") {next.min_forward_cmd = v;}
  }
  rcl_interfaces::msg::SetParametersResult r;
  const std::string bad = next.validate();
  r.successful = bad.empty();
  r.reason = bad;
  if (r.successful) {
    params_ = next;
    governor_.set_params(next);
    speed_.configure(next.speed_window_s, next.max_speed_age_s);
  }
  return r;
}

}  // namespace qazbot_approach_governor

RCLCPP_COMPONENTS_REGISTER_NODE(qazbot_approach_governor::ApproachGovernorNode)
