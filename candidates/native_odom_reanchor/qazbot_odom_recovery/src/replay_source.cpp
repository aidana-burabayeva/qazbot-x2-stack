// C++ high-rate fault injector. Only publishes to private twin02 test topics.
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <aimdk_msgs/msg/mc_common_state.hpp>

class Source : public rclcpp::Node
{
public:
  Source() : Node("twin02_replay_source")
  {
    if (!std::getenv("TWIN_SANDBOX") || std::string(std::getenv("TWIN_SANDBOX")) != "1" ||
      !std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) != "87")
    {throw std::runtime_error("isolated SIM only");}
    gap_ = declare_parameter("gap_s", 1.);
    jump_ = declare_parameter("jump_m", .467);
    announced_ = declare_parameter("announced_reset", true);
    invalid_ = declare_parameter("invalid_after_reset", false);
    if (gap_ < .1 || gap_ > 40 || jump_ < .467 || jump_ > 1.6) {throw std::runtime_error("invalid measured test range");}
    start_ = now().seconds();
    odom_ = create_publisher<nav_msgs::msg::Odometry>("/twin02/leg_source", rclcpp::SensorDataQoS());
    mc_ = create_publisher<aimdk_msgs::msg::McCommonState>("/twin02/mc_state", rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(std::chrono::milliseconds(6), [this] {tick();});
  }
private:
  void tick()
  {
    const double t = now().seconds()-start_;
    const bool paused = t >= 3 && t < 3+gap_, reset = t >= 3+gap_;
    aimdk_msgs::msg::McCommonState mc;
    mc.body_status.value = (paused && announced_) ? 2 : 1;
    mc.fsm_state.current_state = 4;
    mc.action_info.action_desc = (paused && announced_) ? "DAMPING_DEFAULT" : "STAND_DEFAULT";
    mc_->publish(mc);
    if (paused) {return;}
    nav_msgs::msg::Odometry m;
    m.header.stamp = now(); m.header.frame_id = "leg_odom"; m.child_frame_id = "lidar_imu_chest_front";
    m.pose.pose.position.x = reset ? (invalid_ ? NAN : jump_) : 0.; m.pose.pose.position.z = 1.;
    const double half = reset ? .35 : 0., s = std::sqrt(.5);
    // yaw(0 or 0.7) * pitch(-pi/2): same native IMU-axis convention as production guard fixtures.
    m.pose.pose.orientation.x = s*std::sin(half);
    m.pose.pose.orientation.y = -s*std::cos(half);
    m.pose.pose.orientation.z = s*std::sin(half);
    m.pose.pose.orientation.w = s*std::cos(half);
    m.twist.twist.angular.z = 1.78; m.twist.twist.linear.y = .30;
    for (unsigned i=0;i<6;++i) {m.pose.covariance[i*7]=.001;}
    odom_->publish(m);
  }
  double start_{}, gap_{}, jump_{}; bool announced_{}, invalid_{};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<aimdk_msgs::msg::McCommonState>::SharedPtr mc_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv)
{rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Source>());rclcpp::shutdown();}
