#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <nav2_core/controller.hpp>
#include <nav2_core/exceptions.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_costmap_2d/costmap_filters/filter_values.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace qazbot_pose_refine {
double angle(double a) {return std::atan2(std::sin(a),std::cos(a));}

class ApproachWindow : public nav2_core::GoalChecker {
public:
  nav2_core::GoalChecker *inner=nullptr; double radius=.03;
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,const std::string &,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose &p,const geometry_msgs::msg::Pose &g,
                     const geometry_msgs::msg::Twist &v) override {return inner->isGoalReached(p,g,v);}
  bool getTolerances(geometry_msgs::msg::Pose &p,geometry_msgs::msg::Twist &v) override {
    if (!inner->getTolerances(p,v)) return false;
    p.position.x=p.position.y=radius; return true;
  }
};

// Nav2 retains the same current XY/yaw tolerances and additionally requires rest.
class Controller : public nav2_core::Controller {
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController rpp_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  geometry_msgs::msg::PoseStamped goal_;
  geometry_msgs::msg::PoseStamped correction_target_;
  nav_msgs::msg::Path global_path_;
  bool have_goal_=false, aligning_=false, correcting_=false;
  double entry_=.03, exit_=.10, yaw_stop_=.035, max_w_=.5, speed_limit_=1.;
  ApproachWindow approach_;
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
                 const std::shared_ptr<tf2_ros::Buffer> tf,
                 const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    const char *sim=std::getenv("QAZBOT_SIM"), *domain=std::getenv("ROS_DOMAIN_ID"), *local=std::getenv("ROS_LOCALHOST_ONLY");
    if (!sim || std::string(sim)!="1" || !domain || std::string(domain)!="87" || !local || std::string(local)!="1")
      throw std::runtime_error("Pose refinement candidate is restricted to isolated simulation");
    node_=parent.lock(); tf_=tf; costmap_=map;
    if (map->getGlobalFrameID()!="odom") throw std::runtime_error("Refinement requires a stable odom local frame");
    auto parameter=[&](const std::string &key,double value) {
      nav2_util::declare_parameter_if_not_declared(node_,name+"."+key,rclcpp::ParameterValue(value));
      return node_->get_parameter(name+"."+key).as_double();
    };
    entry_=parameter("refine_entry",.03); exit_=parameter("refine_exit",.10);
    yaw_stop_=parameter("refine_yaw_stop",.035); max_w_=parameter("rotate_to_heading_angular_vel",.5);
    if (!(entry_>0 && exit_>entry_ && yaw_stop_>0 && max_w_>0 && max_w_<=.5))
      throw std::runtime_error("Invalid refinement windows/limits");
    approach_.radius=entry_;rpp_.configure(parent,name,tf,map);
  }
  void cleanup() override {rpp_.cleanup(); have_goal_=false; aligning_=false; correcting_=false;}
  void activate() override {rpp_.activate();}
  void deactivate() override {rpp_.deactivate();}
  void setPlan(const nav_msgs::msg::Path &path) override {
    if (path.poses.empty()) throw nav2_core::PlannerException("Empty refinement path");
    auto next=path.poses.back(); next.header.frame_id=path.header.frame_id;
    const bool same=have_goal_ && goal_.header.frame_id==next.header.frame_id &&
      std::hypot(goal_.pose.position.x-next.pose.position.x,goal_.pose.position.y-next.pose.position.y)<1e-6 &&
      std::abs(angle(tf2::getYaw(goal_.pose.orientation)-tf2::getYaw(next.pose.orientation)))<1e-6;
    if (!same) {aligning_=false; correcting_=false;}
    goal_=next; have_goal_=true; global_path_=path;
    if (!correcting_) rpp_.setPlan(path);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed,
      nav2_core::GoalChecker *checker) override {
    if (!have_goal_) throw nav2_core::PlannerException("No refinement goal");
    auto target=goal_; target.header.stamp=pose.header.stamp;
    if (target.header.frame_id!=pose.header.frame_id) {
      try {target=tf_->transform(target,pose.header.frame_id,tf2::durationFromSec(.2));}
      catch (const tf2::TransformException &e) {throw nav2_core::PlannerException(e.what());}
    }
    const double distance=std::hypot(target.pose.position.x-pose.pose.position.x,target.pose.position.y-pose.pose.position.y);
    const double yaw_error=angle(tf2::getYaw(target.pose.orientation)-tf2::getYaw(pose.pose.orientation));
    if (!std::isfinite(distance) || !std::isfinite(yaw_error)) throw nav2_core::PlannerException("Invalid refinement pose");
    // A short correction has a fixed odometry target. Changing the viewing
    // direction must not keep redirecting it through new AMCL corrections.
    if (correcting_) {
      const double remaining=std::hypot(correction_target_.pose.position.x-pose.pose.position.x,
                                       correction_target_.pose.position.y-pose.pose.position.y);
      if (remaining>.01) {
        approach_.inner=checker;approach_.radius=.01;
        return rpp_.computeVelocityCommands(pose,speed,&approach_);
      }
      correcting_=false;aligning_=true;rpp_.setPlan(global_path_);
    }
    if (aligning_ && distance>exit_) aligning_=false;
    if (distance<=entry_) aligning_=true;
    geometry_msgs::msg::Pose tolerance;geometry_msgs::msg::Twist velocity_tolerance;
    if (!checker->getTolerances(tolerance,velocity_tolerance))
      throw nav2_core::PlannerException("Goal tolerance unavailable");
    // Reassess small position corrections only after finishing the heading phase.
    // This does not accept a goal outside the original XY window.
    if (aligning_ && std::abs(yaw_error)<=yaw_stop_ && distance>tolerance.position.x) {
      nav_msgs::msg::Path local;local.header=pose.header;
      correction_target_=target;correction_target_.header=pose.header;
      const double heading=std::atan2(target.pose.position.y-pose.pose.position.y,
                                      target.pose.position.x-pose.pose.position.x);
      tf2::Quaternion q;q.setRPY(0.,0.,heading);
      correction_target_.pose.orientation=tf2::toMsg(q);
      local.poses={pose,correction_target_};
      correcting_=true;aligning_=false;rpp_.setPlan(local);
      approach_.inner=checker;approach_.radius=.01;
      return rpp_.computeVelocityCommands(pose,speed,&approach_);
    }
    if (!aligning_) {
      approach_.inner=checker;approach_.radius=entry_;return rpp_.computeVelocityCommands(pose,speed,&approach_);
    }
    // The configured X2 envelope is circular: a stationary turn has the same
    // swept envelope. Preserve footprint collision checking during alignment.
    auto *map=costmap_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision(map);
    const double footprint_cost=collision.footprintCostAtPose(pose.pose.position.x,pose.pose.position.y,
      tf2::getYaw(pose.pose.orientation),costmap_->getRobotFootprint());
    unsigned int mx,my;
    const bool inside=map->worldToMap(pose.pose.position.x,pose.pose.position.y,mx,my);
    // Inflated radius costs apply at the centre; polygon edges test lethal cells.
    // Treating edge cost 253 as collision would count the body radius twice.
    if (!inside || map->getCost(mx,my)>=nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
        footprint_cost>=nav2_costmap_2d::LETHAL_OBSTACLE || footprint_cost<0.)
      throw nav2_core::PlannerException("Refinement footprint is not clear");
    geometry_msgs::msg::TwistStamped command; command.header=pose.header;
    if (std::abs(yaw_error)>yaw_stop_)
      command.twist.angular.z=std::copysign(std::min(max_w_*speed_limit_,std::abs(yaw_error)),yaw_error);
    return command;
  }
  void setSpeedLimit(const double &limit,const bool &percentage) override {
    rpp_.setSpeedLimit(limit,percentage);
    speed_limit_=limit==nav2_costmap_2d::NO_SPEED_LIMIT ? 1. :
      std::clamp(percentage ? limit/100. : limit/.3,0.,1.);
  }
};
}  // namespace qazbot_pose_refine
PLUGINLIB_EXPORT_CLASS(qazbot_pose_refine::Controller,nav2_core::Controller)
