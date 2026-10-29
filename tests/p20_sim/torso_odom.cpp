// SIM ONLY. Official MC v1.0.0 does not emit native leg odometry here.
// Use MuJoCo pelvis + measured joint FK for a torso-yaw surrogate, never claim native-odom fidelity.
#include <chrono>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class TorsoOdom : public rclcpp::Node
{
  using Odom = nav_msgs::msg::Odometry;
  using TF = tf2_msgs::msg::TFMessage;
  using Transform = geometry_msgs::msg::TransformStamped;
public:
  TorsoOdom() : Node("p20_sim_torso_odom")
  {
    if (!std::getenv("TWIN_SANDBOX") || std::string(std::getenv("TWIN_SANDBOX")) != "1" ||
      !std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) != "87")
    {throw std::runtime_error("isolated SIM only");}
    pub_ = create_publisher<Odom>("/twin/p20_odom", 5);
    hal_ = create_subscription<Odom>("/aima/hal/odom/state", rclcpp::SensorDataQoS(),
      [this](Odom::ConstSharedPtr m) {root_ = m;});
    tf_ = create_subscription<TF>("/tf", rclcpp::SensorDataQoS(), [this](TF::ConstSharedPtr m) {
      for (const auto & t : m->transforms) {graph_[t.child_frame_id] = t;}
    });
    timer_ = create_wall_timer(std::chrono::milliseconds(33), [this] {publish();});
  }
private:
  void publish()
  {
    if (!root_) {return;}
    auto stamp = rclcpp::Time(root_->header.stamp);
    if (stamp == last_ || (now() - stamp).seconds() > .2) {return;}
    const auto & p = root_->pose.pose.position;
    const auto & q = root_->pose.pose.orientation;
    Eigen::Isometry3d world = Eigen::Isometry3d::Identity();
    world.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    world.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    std::vector<Transform> chain;
    std::string child = "torso_link";
    for (unsigned i = 0; child != "pelvis" && i < 10; ++i) {
      const auto it = graph_.find(child);
      if (it == graph_.end()) {return;}
      const auto & t = it->second;
      if (std::abs((stamp - rclcpp::Time(t.header.stamp)).seconds()) > .1) {return;}
      chain.push_back(t); child = t.header.frame_id;
    }
    if (child != "pelvis") {return;}
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
      const auto & t = it->transform.translation;
      const auto & r = it->transform.rotation;
      Eigen::Isometry3d local = Eigen::Isometry3d::Identity();
      local.translation() = Eigen::Vector3d(t.x, t.y, t.z);
      local.linear() = Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized().toRotationMatrix();
      world = world * local;
    }
    const double yaw = std::atan2(world.linear()(1, 0), world.linear()(0, 0));
    Odom out;
    out.header.stamp = root_->header.stamp;
    out.header.frame_id = "odom"; out.child_frame_id = "base_footprint";
    out.pose.pose.position.x = world.translation().x();
    out.pose.pose.position.y = world.translation().y();
    out.pose.pose.position.z = world.translation().z();
    out.pose.pose.orientation.z = std::sin(yaw / 2);
    out.pose.pose.orientation.w = std::cos(yaw / 2);
    out.twist.twist.angular.z = 1.78; out.twist.twist.linear.y = .30; // deliberately invalid, pose-only P20
    pub_->publish(out); last_ = stamp;
  }
  Odom::ConstSharedPtr root_;
  std::unordered_map<std::string, Transform> graph_;
  rclcpp::Time last_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<Odom>::SharedPtr pub_;
  rclcpp::Subscription<Odom>::SharedPtr hal_;
  rclcpp::Subscription<TF>::SharedPtr tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorsoOdom>());
  rclcpp::shutdown();
}
