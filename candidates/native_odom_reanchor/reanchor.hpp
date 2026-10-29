#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

// SIM-ONLY candidate. ROS transport and reset-evidence acquisition deliberately stay outside this core.
namespace qazbot::candidate
{
struct Pose { double x{}, y{}, yaw{}; };
struct Sample { double stamp{}; Pose pose; };
enum class State { tracking, suspended, verifying, latched };

inline double wrap(double x) { return std::remainder(x, 2.0 * std::acos(-1.0)); }
inline bool finite(const Pose & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
}
inline Pose compose(const Pose & a, const Pose & b)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, wrap(a.yaw + b.yaw)};
}
inline Pose inverse(const Pose & p)
{
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  return {-c * p.x - s * p.y, s * p.x - c * p.y, -p.yaw};
}

class Reanchor
{
public:
  State state() const { return state_; }
  std::uint64_t epoch() const { return epoch_; }
  // Same jump limits as production native_odom.py; a long gap cannot widen them.
  std::optional<Sample> update(const Sample & raw, double now)
  {
    if (!std::isfinite(now) || !std::isfinite(raw.stamp) || !finite(raw.pose)) {
      state_ = State::latched;
      return {};
    }
    if (state_ == State::latched || state_ == State::suspended || raw.stamp <= 0 ||
      raw.stamp - now > 0.2 || now - raw.stamp > 0.2)
    {
      return {};
    }
    if (previous_) {
      const double dt = raw.stamp - previous_->stamp;
      if (dt <= 0) {return {};}
      if (std::hypot(raw.pose.x - previous_->pose.x, raw.pose.y - previous_->pose.y) >
        0.05 + 1.5 * std::min(dt, 0.2) ||
        std::abs(wrap(raw.pose.yaw - previous_->pose.yaw)) > 0.3)
      {
        state_ = State::latched;
        return {};
      }
    }
    previous_ = raw;
    output_ = Sample{raw.stamp, compose(offset_, raw.pose)};
    ++fresh_samples_;
    return output_;
  }

  // An observed, expected MC stand->damping transition only SUSPENDS output. It is not reset proof.
  bool begin_reset(std::uint64_t epoch, bool stand_to_damping, bool motion_inhibited)
  {
    if (state_ != State::tracking || !output_ || epoch <= epoch_ ||
      !stand_to_damping || !motion_inhibited)
    {
      return false;
    }
    epoch_ = epoch;
    state_ = State::suspended;
    fresh_samples_ = 0;
    return true;
  }

  // verified is an independently measured pose in the OLD published odom frame, matched to raw.stamp.
  // Never fill it with the last pose merely because MC changed mode: the robot may have moved during the gap.
  bool confirm_reset(std::uint64_t epoch, const Sample & raw, const Sample & verified,
    double now, bool source_reset_confirmed, bool mc_standing, bool motion_inhibited)
  {
    if (state_ != State::suspended || epoch != epoch_ || !source_reset_confirmed ||
      !mc_standing || !motion_inhibited || !finite(raw.pose) || !finite(verified.pose) ||
      !std::isfinite(now) || !std::isfinite(raw.stamp) || !std::isfinite(verified.stamp) ||
      raw.stamp <= 0 || verified.stamp <= 0 || now - raw.stamp > 0.2 ||
      raw.stamp > now || std::abs(raw.stamp - verified.stamp) > 0.001)
    {
      return false;
    }
    offset_ = compose(verified.pose, inverse(raw.pose));
    previous_ = raw;
    output_.reset();
    fresh_samples_ = 0;
    state_ = State::verifying;
    return true;
  }

  // New odom is available to localisation while motion stays inhibited. Only explicit fresh Trust
  // requalification releases the interlock. Old READY alone must never be forwarded here.
  bool release(std::uint64_t epoch, bool newly_qualified_trust, double now)
  {
    if (state_ != State::verifying || epoch != epoch_ || !newly_qualified_trust ||
      fresh_samples_ < 2 || !fresh(now))
    {
      return false;
    }
    state_ = State::tracking;
    return true;
  }
  bool motion_allowed(double now, bool trust_motion_ok) const
  {
    return state_ == State::tracking && trust_motion_ok && fresh(now);
  }

private:
  bool fresh(double now) const
  {
    return output_ && std::isfinite(now) && now >= output_->stamp && now - output_->stamp <= 0.2;
  }
  State state_{State::tracking};
  std::uint64_t epoch_{};
  Pose offset_{};
  std::optional<Sample> previous_, output_;
  unsigned fresh_samples_{};
};
}  // namespace qazbot::candidate
