#pragma once
#include <cmath>

namespace qazbot_mc_turn {
// FollowPath can reset its checker for each refreshed plan. A dwell belongs to
// the physical goal/condition, not to an unchanged path message.
class GoalDwell {
  double since_=-1,last_=-1,x_=0,y_=0,yaw_=0;
 public:
  bool update(double now,bool valid,double x,double y,double yaw) {
    const double angle=std::atan2(std::sin(yaw-yaw_),std::cos(yaw-yaw_));
    if(last_<0 || now<last_ || now-last_>.25 || std::hypot(x-x_,y-y_)>.001 || std::abs(angle)>.001)since_=-1;
    last_=now;x_=x;y_=y;yaw_=yaw;
    if(!valid){since_=-1;return false;}
    if(since_<0)since_=now;
    return now-since_>=3.;
  }
};
}
