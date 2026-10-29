#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <aimdk_msgs/msg/mc_locomotion_velocity.hpp>

class Recorder : public rclcpp::Node
{
public:
  Recorder() : Node("p20_timing_recorder")
  {
    if (!std::getenv("TWIN_SANDBOX") || std::string(std::getenv("TWIN_SANDBOX")) != "1" ||
      !std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID")) != "87")
    {throw std::runtime_error("isolated SIM only");}
    const auto output = declare_parameter<std::string>("output_dir", "");
    if (output.empty() || std::filesystem::exists(output)) {
      throw std::runtime_error("output_dir must be new and nonempty");
    }
    std::filesystem::create_directories(output);
    motion_.open(output + "/motion.csv"); status_.open(output + "/status.jsonl");
    if (!motion_ || !status_) {throw std::runtime_error("cannot open evidence files");}
    motion_ << "received_ros_ns,received_steady_ns,source_stamp_ns,kind,x,y,yaw,vx,vy,wz,source\n" << std::setprecision(17);
    for (const auto & item : std::vector<std::pair<std::string, std::string>>{
      {"truth", "/x2_twin/model_truth"}, {"torso", "/qazbot_native/odom"}})
    {
      odoms_.push_back(create_subscription<nav_msgs::msg::Odometry>(item.second, rclcpp::SensorDataQoS(),
        [this, kind=item.first](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          const auto & p=m->pose.pose.position; const auto & q=m->pose.pose.orientation;
          const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
          row(kind, rclcpp::Time(m->header.stamp).nanoseconds(), p.x,p.y,yaw,
            m->twist.twist.linear.x,m->twist.twist.linear.y,m->twist.twist.angular.z, "");
        }));
    }
    for (const auto & item : std::vector<std::pair<std::string, std::string>>{
      {"nav", "/qazbot_d1_map/cmd_vel_nav"}, {"smoothed", "/qazbot_d1_map/cmd_vel"},
      {"authorized", "/qazbot_d1_map/cmd_vel_authorized"}})
    {
      twists_.push_back(create_subscription<geometry_msgs::msg::Twist>(item.second, 20,
        [this, kind=item.first](geometry_msgs::msg::Twist::ConstSharedPtr m) {
          row(kind,0,NAN,NAN,NAN,m->linear.x,m->linear.y,m->angular.z,"");
        }));
    }
    mc_=create_subscription<aimdk_msgs::msg::McLocomotionVelocity>("/aima/mc/locomotion/velocity",rclcpp::SensorDataQoS(),
      [this](aimdk_msgs::msg::McLocomotionVelocity::ConstSharedPtr m) {
        row("mc",rclcpp::Time(m->header.stamp).nanoseconds(),NAN,NAN,NAN,
          m->forward_velocity,m->lateral_velocity,m->angular_velocity,m->source);
      });
    for (const auto & item : std::vector<std::pair<std::string,std::string>>{
      {"adapter","/qazbot_d1_map/motion_adapter/status"}, {"coordinator","/qazbot_d1_map/goal_coordinator/status"}})
    {
      statuses_.push_back(create_subscription<std_msgs::msg::String>(item.second,10,
        [this, kind=item.first](std_msgs::msg::String::ConstSharedPtr m) {
          status_ << "{\"received_ros_ns\":" << now().nanoseconds() << ",\"kind\":\"" << kind << "\",\"data\":" << m->data << "}\n";
        }));
    }
    timer_=create_wall_timer(std::chrono::seconds(1),[this]{motion_.flush();status_.flush();});
    RCLCPP_INFO(get_logger(),"Recording P20 phase evidence to %s",output.c_str());
  }
private:
  void row(const std::string & kind, int64_t stamp, double x,double y,double yaw,double vx,double vy,double wz,const std::string & source)
  {
    const auto steady=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    motion_ << now().nanoseconds() << ',' << steady << ',' << stamp << ',' << kind << ',' << x << ',' << y << ','
      << yaw << ',' << vx << ',' << vy << ',' << wz << ',' << source << '\n';
  }
  std::ofstream motion_,status_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odoms_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> twists_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> statuses_;
  rclcpp::Subscription<aimdk_msgs::msg::McLocomotionVelocity>::SharedPtr mc_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv)
{rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Recorder>());rclcpp::shutdown();}
