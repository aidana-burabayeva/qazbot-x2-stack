#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace qazbot_mc_turn {
// Brake from observed velocity, then observe the complete stop before either
// admitting the final rotation or trying another bounded forward approach.
class WalkBrake {
  bool holding_=false;
  double zero_at_=0;
  int stops_=0;
 public:
  enum Result {Pass, Hold};
  void reset(){holding_=false;stops_=0;}
  bool holding()const{return holding_;}
  Result update(double now,double distance,double forward,double measured_speed,
                double tolerance,double lead,bool translating) {
    if(!std::isfinite(now+distance+forward+measured_speed+tolerance+lead) || tolerance<=0 || lead<0)
      throw std::runtime_error("invalid approach observation");
    if(!holding_) {
      if(!translating || distance>.65 || distance>std::max(tolerance*.65,std::max(0.,measured_speed)*lead+tolerance*.35))return Pass;
      holding_=true;zero_at_=now;
      if(++stops_>4)throw std::runtime_error("MC approach exceeded four braking attempts");
    }
    if(now-zero_at_<3. || std::abs(measured_speed)>.025)return Hold;
    if(distance<=tolerance*.8)return Pass;
    // A missed endpoint may require a bounded reorientation before the next
    // forward approach. The rotation controller and stop count bound that retry.
    holding_=false;return Pass;
  }
};
}
