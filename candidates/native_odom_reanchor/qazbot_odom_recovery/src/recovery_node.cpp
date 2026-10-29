#include "../../reanchor.hpp"
#include <array>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <aimdk_msgs/msg/mc_common_state.hpp>
#include <qazbot_odom_recovery_msgs/srv/recover_odometry.hpp>

namespace qc = qazbot::candidate;
class RecoveryNode : public rclcpp::Node
{
  using Odom = nav_msgs::msg::Odometry;
  using Mc = aimdk_msgs::msg::McCommonState;
  using Recovery = qazbot_odom_recovery_msgs::srv::RecoverOdometry;
public:
  RecoveryNode() : Node("twin02_recovery")
  {
    if (!std::getenv("TWIN_SANDBOX") || std::string(std::getenv("TWIN_SANDBOX")) != "1" ||
      !std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) != "87")
    {throw std::runtime_error("SIM ONLY: TWIN_SANDBOX=1 and ROS_DOMAIN_ID=87 required");}
    odom_pub_ = create_publisher<Odom>("/twin02/odom", 10);
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/twin02/status", 10);
    mc_sub_ = create_subscription<Mc>("/twin02/mc_state", rclcpp::SensorDataQoS(), [this](Mc::ConstSharedPtr m) {
      const auto next = m->action_info.action_desc;
      if (next != action_) {previous_action_ = action_; action_ = next;}
      stand_ = m->body_status.value == 1 && (m->fsm_state.current_state == 4 || m->fsm_state.current_state == 5);
      mc_time_ = now().seconds();
    });
    odom_sub_ = create_subscription<Odom>("/twin02/leg_source", rclcpp::SensorDataQoS(),
      [this](Odom::ConstSharedPtr m) {on_odom(m);});
    service_ = create_service<Recovery>("/twin02/recover",
      [this](const Recovery::Request::SharedPtr req, Recovery::Response::SharedPtr res) {recover(*req, *res);});
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {status();});
  }
private:
  static bool quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    return std::isfinite(norm) && std::abs(norm - 1.) <= .01;
  }
  void on_odom(Odom::ConstSharedPtr m)
  {
    if (m->header.frame_id != "leg_odom" || m->child_frame_id != "lidar_imu_chest_front") {return;}
    const auto & p = m->pose.pose.position;
    const auto & q = m->pose.pose.orientation;
    if (!quaternion(q) || !std::isfinite(p.z)) {
      core_.update({now().seconds(), {NAN, 0, 0}}, now().seconds()); return;
    }
    // Exact production native_odom.imu_footprint_yaw: rotate the LiDAR-forward vector then project to XY.
    constexpr std::array<double, 3> f{-0.003843965609489519, 0.0008810920694298785, -0.9999922237723444};
    const double fx = (1-2*(q.y*q.y+q.z*q.z))*f[0] + 2*(q.x*q.y-q.z*q.w)*f[1] + 2*(q.x*q.z+q.y*q.w)*f[2];
    const double fy = 2*(q.x*q.y+q.z*q.w)*f[0] + (1-2*(q.x*q.x+q.z*q.z))*f[1] + 2*(q.y*q.z-q.x*q.w)*f[2];
    if (std::hypot(fx, fy) < .5) {return;}
    qc::Sample raw{rclcpp::Time(m->header.stamp).seconds(), {p.x, p.y, std::atan2(fy, fx)}};
    if (!qc::finite(raw.pose)) {core_.update(raw, now().seconds()); return;}
    if (raw.stamp <= 0 || std::abs(now().seconds()-raw.stamp) > .2) {return;}
    raw_ = raw;
    raw_history_.push_back(raw);
    while (raw_history_.size() > 50) {raw_history_.pop_front();}
    const auto output = core_.update(raw, now().seconds());
    if (!output) {return;}
    if (output->stamp - last_pub_ < 1./30.) {return;}
    auto out = *m;
    out.header.frame_id = "odom"; out.child_frame_id = "base_footprint";
    out.pose.pose.position.x = output->pose.x; out.pose.pose.position.y = output->pose.y;
    out.pose.pose.orientation.x = out.pose.pose.orientation.y = 0;
    out.pose.pose.orientation.z = std::sin(output->pose.yaw/2);
    out.pose.pose.orientation.w = std::cos(output->pose.yaw/2);
    // Pose-only candidate: no claim of a valid twist through reset/gap.
    out.twist = geometry_msgs::msg::TwistWithCovariance();
    for (unsigned i=0; i<6; ++i) {out.twist.covariance[i*7] = 1.e6;}
    odom_pub_->publish(out); last_pub_ = output->stamp;
  }
  void recover(const Recovery::Request & req, Recovery::Response & res)
  {
    const double t = now().seconds();
    const bool mc_fresh = t >= mc_time_ && t-mc_time_ <= .5;
    bool success = false;
    if (req.command == Recovery::Request::BEGIN) {
      success = core_.begin_reset(req.epoch, mc_fresh && previous_action_ == "STAND_DEFAULT" &&
        action_ == "DAMPING_DEFAULT", req.motion_inhibited);
    } else if (req.command == Recovery::Request::CONFIRM && raw_ &&
      req.verified_pose.header.frame_id == "odom" && quaternion(req.verified_pose.pose.orientation))
    {
      const auto & p = req.verified_pose.pose.position;
      const auto & q = req.verified_pose.pose.orientation;
      qc::Sample verified{rclcpp::Time(req.verified_pose.header.stamp).seconds(),
        {p.x, p.y, std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}};
      auto matching = raw_history_.end();
      for (auto it = raw_history_.begin(); it != raw_history_.end(); ++it) {
        if (std::abs(it->stamp - verified.stamp) <= .001) {matching = it; break;}
      }
      if (matching == raw_history_.end()) {res.success=false; res.message="no source sample at verified stamp"; return;}
      success = core_.confirm_reset(req.epoch, *matching, verified, t,
        req.source_reset_confirmed, mc_fresh && stand_ && action_ == "STAND_DEFAULT", req.motion_inhibited);
    } else if (req.command == Recovery::Request::RELEASE) {
      success = core_.release(req.epoch, req.newly_qualified_trust && mc_fresh && stand_, t);
    }
    res.success = success;
    res.state = static_cast<unsigned char>(core_.state());
    res.message = success ? "accepted" : "recovery evidence or state rejected";
    status();
  }
  void status()
  {
    diagnostic_msgs::msg::DiagnosticArray a; a.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus d;
    d.name = "twin02/odom_recovery"; d.hardware_id = "simulation_only";
    const bool gate = core_.motion_allowed(now().seconds(), true) && stand_ && now().seconds()-mc_time_ <= .5;
    d.level = gate ? d.OK : d.ERROR;
    d.message = gate ? "recovery gate open; external Trust still required" : "motion inhibited";
    for (const auto & item : std::array<std::pair<std::string, std::string>, 3>{
      std::make_pair("state", std::to_string(static_cast<int>(core_.state()))),
      std::make_pair("epoch", std::to_string(core_.epoch())),
      std::make_pair("recovery_gate", gate ? "open" : "closed")})
    {diagnostic_msgs::msg::KeyValue v; v.key = item.first; v.value = item.second; d.values.push_back(v);}
    a.status.push_back(d); status_pub_->publish(a);
  }
  qc::Reanchor core_;
  std::optional<qc::Sample> raw_;
  std::deque<qc::Sample> raw_history_;
  double last_pub_{-1e9}, mc_time_{-1e9};
  bool stand_{};
  std::string action_, previous_action_;
  rclcpp::Publisher<Odom>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<Mc>::SharedPtr mc_sub_;
  rclcpp::Service<Recovery>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<RecoveryNode>()); rclcpp::shutdown();
}
