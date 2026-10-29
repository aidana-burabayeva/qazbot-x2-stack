#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <nav2_controller/plugins/stopped_goal_checker.hpp>
#include <nav2_util/node_utils.hpp>
#include <nav2_core/exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include "goal_dwell.hpp"
#include "walk_brake.hpp"
#include "goal_identity.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace qazbot_mc_turn {
void guard() {
  // Twin copy of the candidate: the original demanded ROS_LOCALHOST_ONLY=1 (one container). The twin's isolation is the internal
  // docker network + TWIN_SANDBOX=1 + domain 87, so those are required instead.
  const auto equal=[](const char *key,const char *value){const char *v=std::getenv(key);return v && std::string(v)==value;};
  if (!equal("QAZBOT_SIM","1") || !equal("ROS_DOMAIN_ID","87") || !equal("TWIN_SANDBOX","1"))
    throw std::runtime_error("Experimental MC turn controller requires the isolated twin (QAZBOT_SIM=1, domain 87, TWIN_SANDBOX=1)");
}
double wrap(double x){return std::atan2(std::sin(x),std::cos(x));}

class Controller : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
  using RPP=nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;
  bool active_=false,settling_=false,done_=false,final_=false;
  double zero_at_=0,started_=0,target_=0,speed_=.8,lead_=.20,tolerance_=.06;
  int attempts_=0;
  WalkBrake brake_;bool walk_enabled_=false;double walk_lead_=.45;
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    guard();RPP::configure(parent,name,tf,costmap);
    const auto n=parent.lock();
    for (auto p: {std::pair<std::string,double>{"mc_turn_speed",.8},{"mc_turn_lead_time",.20},{"mc_turn_tolerance",.06}})
      nav2_util::declare_parameter_if_not_declared(n,name+"."+p.first,rclcpp::ParameterValue(p.second));
    n->get_parameter(name+".mc_turn_speed",speed_);n->get_parameter(name+".mc_turn_lead_time",lead_);n->get_parameter(name+".mc_turn_tolerance",tolerance_);
    nav2_util::declare_parameter_if_not_declared(n,name+".mc_walk_brake",rclcpp::ParameterValue(false));
    nav2_util::declare_parameter_if_not_declared(n,name+".mc_walk_lead_time",rclcpp::ParameterValue(.45));
    n->get_parameter(name+".mc_walk_brake",walk_enabled_);n->get_parameter(name+".mc_walk_lead_time",walk_lead_);
    if(!std::isfinite(walk_lead_) || walk_lead_<0 || walk_lead_>1.5)throw std::runtime_error("invalid walk lead");
    if(speed_<=0 || speed_>1 || lead_<0 || tolerance_<=0)throw std::runtime_error("invalid turn parameters");
  }
  void setPlan(const nav_msgs::msg::Path &path) override {
    if(!global_plan_.poses.empty() && !path.poses.empty()) {
      const auto &a=global_plan_.poses.back().pose;const auto &b=path.poses.back().pose;
      if(std::hypot(a.position.x-b.position.x,a.position.y-b.position.y)>.05 ||
         std::abs(wrap(tf2::getYaw(a.orientation)-tf2::getYaw(b.orientation)))>.05) {active_=false;brake_.reset();}
    }
    RPP::setPlan(path);
  }
  void deactivate() override {active_=false;brake_.reset();RPP::deactivate();}
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity,nav2_core::GoalChecker *goal) override {
    auto tracked_goal=dynamic_cast<GoalIdentitySink*>(goal);
    if(tracked_goal) {
      std::lock_guard<std::mutex> lock(mutex_);
      if(!global_plan_.poses.empty()) {
        const auto &end=global_plan_.poses.back();
        tracked_goal->setTarget(global_plan_.header.frame_id,end.pose.position.x,end.pose.position.y,tf2::getYaw(end.pose.orientation));
      }
    }
    auto command=RPP::computeVelocityCommands(pose,velocity,goal);
    std::lock_guard<std::mutex> lock(mutex_);
    auto plan=transformGlobalPlan(pose);
    if(plan.poses.empty())return command;
    auto carrot=getLookAheadPoint(getLookAheadDistance(velocity),plan);
    double path_error=0;
    const bool goal_rotation=shouldRotateToGoalHeading(carrot);
    bool path_rotation=shouldRotateToPath(carrot,path_error);
    if(walk_enabled_ && !active_) {
      const auto &end=plan.poses.back().pose.position;
      geometry_msgs::msg::Pose pose_tol;geometry_msgs::msg::Twist vel_tol;
      if(!goal->getTolerances(pose_tol,vel_tol))throw nav2_core::PlannerException("missing approach tolerances");
      const bool moving=command.twist.linear.x>0 || velocity.linear.x>.025;
      const bool was_holding=brake_.holding();
      if(brake_.update(clock_->now().seconds(),std::hypot(end.x,end.y),end.x,
          velocity.linear.x,pose_tol.position.x,walk_lead_,moving)==WalkBrake::Hold) {
        if(!was_holding)RCLCPP_INFO(logger_,"MC walk brake distance=%.3f measured_speed=%.3f lead=%.3f",std::hypot(end.x,end.y),velocity.linear.x,walk_lead_);
        command.twist=geometry_msgs::msg::Twist{};return command;
      }
      if(was_holding && !brake_.holding())RCLCPP_INFO(logger_,"MC walk retry distance=%.3f forward=%.3f",std::hypot(end.x,end.y),end.x);
      if(!goal_rotation && std::hypot(end.x,end.y)<.65 && std::abs(std::atan2(end.y,end.x))>.25) {
        path_rotation=true;carrot.pose.position=end;
      }
    }
    if(!active_ && !goal_rotation && !path_rotation)return command;
    // RPP supplies the target geometry; the regulator does not integrate requested velocity.
    double error=(goal_rotation || (active_ && final_))?tf2::getYaw(plan.poses.back().pose.orientation):std::atan2(carrot.pose.position.y,carrot.pose.position.x);
    const double now=clock_->now().seconds(),heading=tf2::getYaw(pose.pose.orientation);
    const double proposed=wrap(heading+error);
    if(!active_) {
      active_=true;settling_=false;done_=false;attempts_=1;started_=now;target_=proposed;final_=goal_rotation;
      RCLCPP_INFO(logger_,"MC turn start error=%.3f final=%d",error,final_);
    } else if(std::abs(wrap(proposed-target_))>.35) {
      active_=false;command.twist=geometry_msgs::msg::Twist{};return command;
    }
    target_=proposed;error=wrap(target_-heading);
    // Braking can aim for a smaller error, but do not kick a settled robot out
    // of the actual goal checker's tolerance while it accumulates its dwell.
    const double accepted=(final_ && tracked_goal)?tracked_goal->yawAcceptance():tolerance_;
    if(now-started_>20)throw nav2_core::PlannerException("MC turn exceeded bounded execution time");
    if(done_) {
      if(std::abs(error)>accepted){done_=false;settling_=true;zero_at_=now-3.;}
      else {
        command.twist=geometry_msgs::msg::Twist{};
        if(!final_ || !goal_rotation)active_=false;
        return command;
      }
    }
    double output=0;
    if(!settling_) {
      const double direction=error>=0?1.:-1.;
      const double stopping=std::max(0.,direction*velocity.angular.z)*lead_+tolerance_*.5;
      if(std::abs(error)<=std::max(tolerance_,stopping)){settling_=true;zero_at_=now;RCLCPP_INFO(logger_,"MC turn brake error=%.3f measured_rate=%.3f",error,velocity.angular.z);}
      else output=direction*speed_;
    } else if(now-zero_at_>=3. && std::abs(velocity.angular.z)<.03) {
      if(std::abs(error)<=accepted){done_=true;RCLCPP_INFO(logger_,"MC turn settled error=%.3f attempts=%d",error,attempts_);}
      else if(attempts_>=4)throw nav2_core::PlannerException("MC turn did not settle within angle tolerance");
      else {++attempts_;settling_=false;RCLCPP_INFO(logger_,"MC turn correction error=%.3f attempt=%d",error,attempts_);output=std::copysign(speed_,error);}
    }
    command.twist=geometry_msgs::msg::Twist{};command.twist.angular.z=output;
    // The stock RPP collision prediction was for its original command. Check the actual candidate too.
    if(output!=0 && isCollisionImminent(pose,0.,output,std::hypot(carrot.pose.position.x,carrot.pose.position.y)))
      throw nav2_core::PlannerException("MC turn candidate collision");
    return command;
  }
};

class SettledGoal : public nav2_controller::StoppedGoalChecker,public GoalIdentitySink {
  GoalDwell dwell_; rclcpp::Clock::SharedPtr clock_;
  std::string frame_;double x_=0,y_=0,yaw_=0;bool target_valid_=false;
 public:
  void setTarget(const std::string &frame,double x,double y,double yaw) override {
    target_valid_=!frame.empty() && std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw);
    if(frame!=frame_ || !target_valid_)dwell_=GoalDwell{};
    frame_=frame;x_=x;y_=y;yaw_=yaw;
  }
  double yawAcceptance()const override{return yaw_goal_tolerance_;}
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,const std::string &name,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    guard();nav2_controller::StoppedGoalChecker::initialize(parent,name,costmap);clock_=parent.lock()->get_clock();
  }
  void reset() override {nav2_controller::StoppedGoalChecker::reset();}
  bool isGoalReached(const geometry_msgs::msg::Pose &query,const geometry_msgs::msg::Pose &goal,
      const geometry_msgs::msg::Twist &velocity) override {
    const bool valid=nav2_controller::StoppedGoalChecker::isGoalReached(query,goal,velocity);
    return dwell_.update(clock_->now().seconds(),valid && target_valid_,x_,y_,yaw_);
  }
};
}
PLUGINLIB_EXPORT_CLASS(qazbot_mc_turn::Controller,nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(qazbot_mc_turn::SettledGoal,nav2_core::GoalChecker)
