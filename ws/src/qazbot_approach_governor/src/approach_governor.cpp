#include "qazbot_approach_governor/approach_governor.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace qazbot_approach_governor
{

std::string Params::validate() const
{
  auto bad = [](double v) {return !std::isfinite(v);};
  if (bad(stop_time_s) || stop_time_s < 0.0 || stop_time_s > 2.0) {return "stop_time_s must be in [0, 2]";}
  if (bad(stop_margin_m) || stop_margin_m < 0.0 || stop_margin_m > 0.5) {return "stop_margin_m must be in [0, 0.5]";}
  if (bad(speed_window_s) || speed_window_s < 0.1 || speed_window_s > 2.0) {return "speed_window_s must be in [0.1, 2]";}
  if (bad(max_speed_age_s) || max_speed_age_s <= 0.0) {return "max_speed_age_s must be > 0";}
  if (bad(max_plan_age_s) || max_plan_age_s <= 0.0) {return "max_plan_age_s must be > 0";}
  if (bad(max_pose_age_s) || max_pose_age_s <= 0.0) {return "max_pose_age_s must be > 0";}
  if (bad(stopped_speed_mps) || stopped_speed_mps <= 0.0 || stopped_speed_mps > 0.3) {
    return "stopped_speed_mps must be in (0, 0.3]";
  }
  if (bad(max_brake_hold_s) || max_brake_hold_s <= 0.0) {return "max_brake_hold_s must be > 0";}
  if (bad(min_forward_cmd) || min_forward_cmd <= 0.0) {return "min_forward_cmd must be > 0";}
  if (bad(max_plausible_speed_mps) || max_plausible_speed_mps < 0.5 || max_plausible_speed_mps > 3.0) {
    return "max_plausible_speed_mps must be in [0.5, 3]";
  }
  return "";
}

SpeedEstimator::SpeedEstimator(double window_s, double max_age_s)
: window_s_(window_s), max_age_s_(max_age_s) {}

void SpeedEstimator::configure(double window_s, double max_age_s)
{
  window_s_ = window_s;
  max_age_s_ = max_age_s;
}

void SpeedEstimator::reset() {samples_.clear();}

void SpeedEstimator::push(double t, double x, double y)
{
  if (!std::isfinite(t) || !std::isfinite(x) || !std::isfinite(y)) {return;}
  if (!samples_.empty() && t <= std::get<0>(samples_.back())) {
    // Time went backwards (clock reset) or duplicate stamp: a duplicate is ignored, a jump restarts the estimate.
    if (t < std::get<0>(samples_.back()) - 1e-6) {samples_.clear();} else {return;}
  }
  samples_.emplace_back(t, x, y);
  while (samples_.size() > 2 && t - std::get<0>(samples_[1]) >= window_s_) {samples_.pop_front();}
}

std::optional<double> SpeedEstimator::speed(double now) const
{
  if (samples_.size() < 2) {return std::nullopt;}
  const auto & a = samples_.front();
  const auto & b = samples_.back();
  if (now - std::get<0>(b) > max_age_s_) {return std::nullopt;}
  const double dt = std::get<0>(b) - std::get<0>(a);
  if (dt < 0.8 * window_s_) {return std::nullopt;}  // history too short (start of stream or after a gap)
  return std::hypot(std::get<1>(b) - std::get<1>(a), std::get<2>(b) - std::get<2>(a)) / dt;
}

void PathProgress::set_path(const std::vector<Point2> & path, double t_received)
{
  path_.clear();
  cum_.clear();
  for (const auto & p : path) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return;}  // refuse a corrupt plan entirely
  }
  for (const auto & p : path) {
    if (!path_.empty() && std::hypot(p.x - path_.back().x, p.y - path_.back().y) < 1e-6) {continue;}
    cum_.push_back(path_.empty() ? 0.0 : cum_.back() + std::hypot(p.x - path_.back().x, p.y - path_.back().y));
    path_.push_back(p);
  }
  if (path_.size() < 2) {path_.clear(); cum_.clear();}
  t_received_ = t_received;
  last_s_ = -1.0;
}

void PathProgress::clear() {path_.clear(); cum_.clear(); last_s_ = -1.0;}

std::optional<double> PathProgress::remaining(const Point2 & r) const
{
  if (!has_path() || !std::isfinite(r.x) || !std::isfinite(r.y)) {return std::nullopt;}
  auto search = [&](double s_lo, double s_hi, double & best_d2, double & best_s) {
      best_d2 = 1e300;
      best_s = 0.0;
      for (size_t i = 0; i + 1 < path_.size(); ++i) {
        if (cum_[i + 1] < s_lo || cum_[i] > s_hi) {continue;}
        const double dx = path_[i + 1].x - path_[i].x, dy = path_[i + 1].y - path_[i].y;
        const double len2 = dx * dx + dy * dy;
        double u = ((r.x - path_[i].x) * dx + (r.y - path_[i].y) * dy) / len2;
        u = std::clamp(u, 0.0, 1.0);
        const double px = path_[i].x + u * dx, py = path_[i].y + u * dy;
        const double d2 = (r.x - px) * (r.x - px) + (r.y - py) * (r.y - py);
        if (d2 < best_d2 - 1e-12) {best_d2 = d2; best_s = cum_[i] + u * std::sqrt(len2);}
      }
    };
  double best_d2 = 1e300, best_s = 0.0;
  const double kBack = 0.5, kAhead = 2.0, kFar = 0.6;  // window around the previous fix [m]
  if (last_s_ >= 0.0) {
    search(last_s_ - kBack, last_s_ + kAhead, best_d2, best_s);
    if (best_d2 > kFar * kFar) {search(-1e300, 1e300, best_d2, best_s);}  // lost: look at the whole path
  } else {
    search(-1e300, 1e300, best_d2, best_s);
  }
  last_s_ = best_s;
  return std::max(0.0, cum_.back() - best_s);
}

ApproachGovernor::ApproachGovernor(const Params & p) : p_(p) {}

void ApproachGovernor::reset() {latched_ = false; latch_time_ = 0.0;}

Decision ApproachGovernor::update(
  double now, const Command & cmd, std::optional<double> speed, std::optional<double> remaining)
{
  Decision d;
  d.out = cmd;
  if (speed && *speed > p_.max_plausible_speed_mps) {  // glitch of the estimate: never act on it
    speed = std::nullopt;
    d.no_data = NoDataReason::kSpeedImplausible;
  }
  if (speed) {d.speed = *speed;}
  if (remaining) {d.remaining = *remaining;}
  const bool forward = cmd.vx >= p_.min_forward_cmd;

  // A latch lives only while the robot is (or may still be) moving. Release: stopped, or hold time exceeded.
  if (latched_) {
    const bool stopped = speed && *speed < p_.stopped_speed_mps;
    if (stopped || now - latch_time_ > p_.max_brake_hold_s) {latched_ = false;}
  }

  if (!forward) {
    // Zero, reverse, pure rotation: never modified. An existing latch keeps running until the robot has stopped.
    d.state = latched_ ? State::kBraking : State::kPassThrough;
    return d;
  }

  if (speed) {d.stop_dist = *speed * p_.stop_time_s + p_.stop_margin_m;}

  if (latched_) {  // still coasting after a brake decision: keep vx at zero (never re-accelerate on the shrinking estimate)
    d.out.vx = 0.0;
    d.state = State::kBraking;
    d.modified = true;
    return d;
  }

  if (!speed || !remaining) {
    d.state = State::kNoData;
    if (d.no_data == NoDataReason::kNone) {d.no_data = !speed ? NoDataReason::kNoSpeed : NoDataReason::kNoRemaining;}
    return d;
  }

  if (*remaining <= d.stop_dist) {
    if (*speed >= p_.stopped_speed_mps) {
      latched_ = true;
      latch_time_ = now;
      d.state = State::kBraking;
    } else {
      d.state = State::kHeldAtStop;  // stopped inside the margin: creeping further would only overshoot
    }
    d.out.vx = 0.0;
    d.modified = true;
    return d;
  }
  d.state = State::kPassThrough;
  return d;
}

}  // namespace qazbot_approach_governor
