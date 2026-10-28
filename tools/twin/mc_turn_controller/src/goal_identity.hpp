#pragma once
#include <string>

namespace qazbot_mc_turn {
// Controller and checker exchange the original global-plan endpoint directly.
// A transformed odom endpoint is not a stable identity when AMCL updates TF.
class GoalIdentitySink {
 public:
  virtual ~GoalIdentitySink()=default;
  virtual void setTarget(const std::string &frame,double x,double y,double yaw)=0;
  virtual double yawAcceptance()const=0;
};
}
