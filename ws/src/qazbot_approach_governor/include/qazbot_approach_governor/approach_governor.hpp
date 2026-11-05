// Approach governor core: pure C++17, no ROS (application logic separated from the node, Henki ROS 2 standard).
//
// Problem (E2E-02, D-05): the X2 Motion Controller walks at ~0.5 m/s for any forward command >= ~0.1 m/s, answers
// with ~0.4 s dead time and coasts 0.07-0.31 m after the command goes to zero. Nav2 RPP only commands zero when the
// robot is already inside the goal tolerance, so the robot overshoots the goal, the goal is then behind it, and RPP
// rotates in place to come back (more body drift, new overshoot).
//
// What this component does: it can ONLY shorten the forward command. When the remaining path length is not larger than
// the predicted stopping distance (measured speed * stop_time + margin) it forces vx = 0 and keeps it at zero until the
// robot has actually stopped. vy and wz are never touched (P20/P23 yaw loop, rotate-to-heading stay as they are).
// Missing or stale inputs never add a restriction beyond the one already latched: the command passes unchanged.
#pragma once

#include <deque>
#include <tuple>
#include <optional>
#include <string>
#include <vector>

namespace qazbot_approach_governor
{

struct Params
{
  double stop_time_s{0.40};        // predicted stop distance = speed * stop_time_s + stop_margin_m
  double stop_margin_m{0.0};
  double speed_window_s{0.30};     // displacement window of the speed estimate
  double max_speed_age_s{0.50};    // newest pose sample older than this -> speed unknown
  double max_plan_age_s{3.0};      // plan older than this -> remaining distance unknown
  double max_pose_age_s{0.30};     // robot pose in the plan frame older than this -> remaining distance unknown
  double stopped_speed_mps{0.08};  // below this the robot counts as stopped (brake latch released)
  double max_brake_hold_s{2.0};    // latch released after this long without a stop (fail-open to the old behaviour)
  double min_forward_cmd{0.02};    // vx below this is not a forward command (pure rotation, zero, reverse)

  // Empty string when valid, otherwise the reason (used by the node's parameter callback and by tests).
  std::string validate() const;
};

struct Command
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct Point2
{
  double x{0.0};
  double y{0.0};
};

// Speed from the displacement of a planar pose stream (FAST-LIO odom_level, 10 Hz) over a short window.
// Stamp based (not arrival based). A time jump backwards or a gap longer than the window restarts the estimate.
class SpeedEstimator
{
public:
  explicit SpeedEstimator(double window_s = 0.30, double max_age_s = 0.50);
  void configure(double window_s, double max_age_s);
  void push(double t, double x, double y);
  // Speed [m/s] at time `now` (same clock as the stamps), or nullopt if there is no fresh, long enough history.
  std::optional<double> speed(double now) const;
  void reset();

private:
  double window_s_;
  double max_age_s_;
  std::deque<std::tuple<double, double, double>> samples_;
};

// Remaining path length from the robot to the end of the plan (arc length from the projection onto the polyline).
class PathProgress
{
public:
  // Replaces the plan; `t_received` is the local time at which it arrived. Fewer than 2 distinct points -> no plan.
  void set_path(const std::vector<Point2> & path, double t_received);
  void clear();
  bool has_path() const {return path_.size() >= 2;}
  double received() const {return t_received_;}
  // Remaining length in metres (0 when the robot is at or beyond the end), nullopt without a plan.
  std::optional<double> remaining(const Point2 & robot) const;

private:
  std::vector<Point2> path_;
  std::vector<double> cum_;  // arc length at each vertex
  double t_received_{0.0};
};

enum class State
{
  kPassThrough,  // far from the end, or nothing to act on
  kBraking,      // vx forced to zero (latched until the robot has stopped)
  kHeldAtStop,   // robot stopped, remaining distance <= margin: vx stays zero
  kNoData        // speed or remaining distance unknown, no latch: command passes unchanged
};

struct Decision
{
  Command out;
  State state{State::kPassThrough};
  double speed{-1.0};      // -1: unknown
  double remaining{-1.0};  // -1: unknown
  double stop_dist{-1.0};  // -1: unknown
  bool modified{false};
};

class ApproachGovernor
{
public:
  explicit ApproachGovernor(const Params & p = Params{});
  void set_params(const Params & p) {p_ = p;}
  const Params & params() const {return p_;}
  // `speed` and `remaining` are nullopt when not available. `now` is a monotonically increasing local time.
  Decision update(double now, const Command & cmd, std::optional<double> speed, std::optional<double> remaining);
  bool latched() const {return latched_;}
  void reset();

private:
  Params p_;
  bool latched_{false};
  double latch_time_{0.0};
};

}  // namespace qazbot_approach_governor
