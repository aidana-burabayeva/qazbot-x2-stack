#include <algorithm>
#include <cmath>
#include <iostream>
using std::abs;
namespace geometry_msgs {namespace msg {struct Twist {struct V {double z=0;}; V angular;};}}
class RegulatedPurePursuitController {public: double rotate_to_heading_angular_vel_,max_angular_accel_,control_duration_; void rotateToHeading(double&,double&,const double&,const geometry_msgs::msg::Twist&);};
class VelocitySmoother {public: double smoothing_frequency_; double applyConstraints(double,double,double,double,double);};
void RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * rotate_to_heading_angular_vel_;

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}
double VelocitySmoother::applyConstraints(
  const double v_curr, const double v_cmd,
  const double accel, const double decel, const double eta)
{
  double dv = v_cmd - v_curr;

  double v_component_max;
  double v_component_min;

  // Accelerating if magnitude of v_cmd is above magnitude of v_curr
  // and if v_cmd and v_curr have the same sign (i.e. speed is NOT passing through 0.0)
  // Decelerating otherwise
  if (abs(v_cmd) >= abs(v_curr) && v_curr * v_cmd >= 0.0) {
    v_component_max = accel / smoothing_frequency_;
    v_component_min = -accel / smoothing_frequency_;
  } else {
    v_component_max = -decel / smoothing_frequency_;
    v_component_min = decel / smoothing_frequency_;
  }

  return v_curr + std::clamp(eta * dv, v_component_min, v_component_max);
}
int main() {
 RegulatedPurePursuitController r; r.rotate_to_heading_angular_vel_=0.5;r.max_angular_accel_=1.0;r.control_duration_=1./20.0;
 VelocitySmoother sm; sm.smoothing_frequency_=20.0;
 bool start=true, ramp=true, stop=true; double first=0;
 for(double sign: {-1.,1.}) {
  geometry_msgs::msg::Twist measured; double v=1,w=0; r.rotateToHeading(v,w,sign,measured);
  first=std::abs(w);start=start && v==0 && std::abs(w-sign*r.rotate_to_heading_angular_vel_)<1e-9;
  double actual_cmd=0;
  for(int k=0;k<10;k++) {double prev=actual_cmd;actual_cmd=sm.applyConstraints(actual_cmd,w,1.,-1.5,1.);ramp=ramp && std::abs(actual_cmd-prev)<=.050000001 && std::abs(actual_cmd)<=.500000001;}
  ramp=ramp && std::abs(actual_cmd-sign*.5)<1e-9;
  for(int k=0;k<7;k++) {double prev=actual_cmd;actual_cmd=sm.applyConstraints(actual_cmd,0.,1.,-1.5,1.);stop=stop && std::abs(actual_cmd-prev)<=.075000001;}
  stop=stop && std::abs(actual_cmd)<1e-9;
 }
 std::cout<<"RPP first magnitude="<<first<<" reaches requested cap="<<start<<" smoother ramp="<<ramp<<" stop="<<stop<<"\n";
 return start && ramp && stop ? 0:1;
}
