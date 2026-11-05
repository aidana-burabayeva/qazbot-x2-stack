// Unit tests of the pure core (no ROS). Replay section uses numbers extracted from the recorded E2E-02 bag.
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "qazbot_approach_governor/approach_governor.hpp"

using namespace qazbot_approach_governor;  // NOLINT

namespace
{
SpeedEstimator feed_const(double v, double t0, double t1, double dt = 0.1)
{
  SpeedEstimator e(0.30, 0.50);
  const int n = static_cast<int>(std::lround((t1 - t0) / dt));
  for (int i = 0; i <= n; ++i) {const double t = t0 + i * dt; e.push(t, v * (t - t0), 0.0);}
  return e;
}
}  // namespace

TEST(SpeedEstimator, ConstantVelocity)
{
  auto e = feed_const(0.5, 0.0, 2.0);
  auto s = e.speed(2.0);
  ASSERT_TRUE(s.has_value());
  EXPECT_NEAR(*s, 0.5, 1e-6);
}

TEST(SpeedEstimator, NeedsHistoryAndIsStaleAfterMaxAge)
{
  SpeedEstimator e(0.30, 0.50);
  EXPECT_FALSE(e.speed(0.0).has_value());
  e.push(0.0, 0, 0);
  EXPECT_FALSE(e.speed(0.0).has_value());  // one sample
  e.push(0.1, 0.05, 0);
  EXPECT_FALSE(e.speed(0.1).has_value());  // 0.1 s < 0.8 * window
  e.push(0.2, 0.10, 0);
  e.push(0.3, 0.15, 0);
  ASSERT_TRUE(e.speed(0.3).has_value());
  EXPECT_FALSE(e.speed(0.9).has_value());  // newest sample 0.6 s old
}

TEST(SpeedEstimator, TimeJumpBackwardsRestartsAndDuplicateIgnored)
{
  auto e = feed_const(0.5, 10.0, 11.0);
  e.push(11.0, 99.0, 99.0);  // duplicate stamp: ignored
  EXPECT_NEAR(*e.speed(11.0), 0.5, 1e-6);
  e.push(1.0, 0.0, 0.0);  // clock jump back
  EXPECT_FALSE(e.speed(1.0).has_value());
}

TEST(SpeedEstimator, IgnoresNonFinite)
{
  auto e = feed_const(0.5, 0.0, 1.0);
  e.push(1.1, std::nan(""), 0.0);
  EXPECT_NEAR(*e.speed(1.0), 0.5, 1e-6);
}

TEST(SpeedEstimator, GaitRippleStaysWithinMeasuredNoise)
{
  // 0.5 m/s with +-0.01 m lateral/forward sway at 10 Hz; measured E2E-02 walking sd at w=0.3 s is ~0.055 m/s.
  std::mt19937 g(1);
  std::normal_distribution<double> n(0.0, 0.01);
  SpeedEstimator e(0.30, 0.50);
  double max_err = 0.0;
  for (int i = 0; i < 100; ++i) {
    const double t = i * 0.1;
    e.push(t, 0.5 * t + n(g), n(g));
    if (auto s = e.speed(t)) {max_err = std::max(max_err, std::fabs(*s - 0.5));}
  }
  EXPECT_LT(max_err, 0.2);
}

TEST(PathProgress, StraightAndEnds)
{
  PathProgress p;
  EXPECT_FALSE(p.has_path());
  p.set_path({{0, 0}, {1, 0}, {2, 0}, {3, 0}}, 0.0);
  ASSERT_TRUE(p.has_path());
  EXPECT_NEAR(*p.remaining({0.0, 0.0}), 3.0, 1e-9);
  EXPECT_NEAR(*p.remaining({1.5, 0.2}), 1.5, 1e-9);   // lateral offset ignored
  EXPECT_NEAR(*p.remaining({3.0, 0.0}), 0.0, 1e-9);
  EXPECT_NEAR(*p.remaining({3.4, 0.0}), 0.0, 1e-9);   // beyond the end
  EXPECT_NEAR(*p.remaining({-1.0, 0.0}), 3.0, 1e-9);  // before the start
}

TEST(PathProgress, PolylineArcLength)
{
  PathProgress p;
  p.set_path({{0, 0}, {1, 0}, {1, 1}}, 0.0);
  EXPECT_NEAR(*p.remaining({0.0, 0.0}), 2.0, 1e-9);
  EXPECT_NEAR(*p.remaining({1.0, 0.5}), 0.5, 1e-9);
}

TEST(PathProgress, DegenerateAndCorruptPathsGiveNoPlan)
{
  PathProgress p;
  p.set_path({{1, 1}}, 0.0);
  EXPECT_FALSE(p.has_path());
  p.set_path({{1, 1}, {1, 1}}, 0.0);
  EXPECT_FALSE(p.has_path());
  p.set_path({{0, 0}, {std::nan(""), 0}, {2, 0}}, 0.0);
  EXPECT_FALSE(p.has_path());
  EXPECT_FALSE(p.remaining({0, 0}).has_value());
  p.set_path({{0, 0}, {2, 0}}, 0.0);
  EXPECT_FALSE(p.remaining({std::nan(""), 0}).has_value());
  p.clear();
  EXPECT_FALSE(p.has_path());
}

TEST(Params, Validation)
{
  Params p;
  EXPECT_EQ(p.validate(), "");
  p.stop_time_s = -1;
  EXPECT_NE(p.validate(), "");
  p = Params{};
  p.stopped_speed_mps = 0;
  EXPECT_NE(p.validate(), "");
  p = Params{};
  p.speed_window_s = std::nan("");
  EXPECT_NE(p.validate(), "");
}

TEST(Governor, FarFromGoalPassesUnchanged)
{
  ApproachGovernor g;
  auto d = g.update(0.0, {0.2, 0.0, 0.1}, 0.5, 2.0);
  EXPECT_EQ(d.state, State::kPassThrough);
  EXPECT_FALSE(d.modified);
  EXPECT_DOUBLE_EQ(d.out.vx, 0.2);
  EXPECT_DOUBLE_EQ(d.out.wz, 0.1);
}

TEST(Governor, BrakesWhenRemainingBelowStopDistanceAndKeepsWzVy)
{
  ApproachGovernor g;  // 0.4 s: stop distance at 0.5 m/s = 0.20 m
  auto d = g.update(0.0, {0.12, 0.01, 0.07}, 0.5, 0.19);
  EXPECT_EQ(d.state, State::kBraking);
  EXPECT_TRUE(d.modified);
  EXPECT_DOUBLE_EQ(d.out.vx, 0.0);
  EXPECT_DOUBLE_EQ(d.out.vy, 0.01);
  EXPECT_DOUBLE_EQ(d.out.wz, 0.07);
  EXPECT_NEAR(d.stop_dist, 0.20, 1e-12);
}

TEST(Governor, LatchDoesNotReleaseWhileSpeedFallsThenReleasesWhenStopped)
{
  ApproachGovernor g;
  g.update(0.0, {0.12, 0, 0}, 0.5, 0.19);
  // speed collapses, estimate of stop distance shrinks below the remaining distance: must NOT re-accelerate
  auto d = g.update(0.2, {0.12, 0, 0}, 0.2, 0.15);
  EXPECT_EQ(d.state, State::kBraking);
  EXPECT_DOUBLE_EQ(d.out.vx, 0.0);
  d = g.update(0.4, {0.12, 0, 0}, 0.1, 0.12);
  EXPECT_DOUBLE_EQ(d.out.vx, 0.0);
  EXPECT_TRUE(g.latched());
  // stopped short of the goal: command passes again (creep towards the goal)
  d = g.update(0.8, {0.12, 0, 0}, 0.02, 0.12);
  EXPECT_FALSE(g.latched());
  EXPECT_EQ(d.state, State::kPassThrough);
  EXPECT_DOUBLE_EQ(d.out.vx, 0.12);
}

TEST(Governor, AtGoalWhileStoppedStaysZero)
{
  ApproachGovernor g;
  auto d = g.update(0.0, {0.12, 0, 0}, 0.0, 0.0);
  EXPECT_EQ(d.state, State::kHeldAtStop);
  EXPECT_DOUBLE_EQ(d.out.vx, 0.0);
}

TEST(Governor, RotationReverseAndZeroAreNeverModified)
{
  ApproachGovernor g;
  for (double vx : {0.0, 0.01, -0.2}) {
    auto d = g.update(0.0, {vx, 0.0, -0.6}, 0.5, 0.05);
    EXPECT_FALSE(d.modified);
    EXPECT_DOUBLE_EQ(d.out.vx, vx);
    EXPECT_DOUBLE_EQ(d.out.wz, -0.6);
  }
}

TEST(Governor, MissingInputsPassThroughButLatchHoldsUntilTimeout)
{
  ApproachGovernor g;
  auto d = g.update(0.0, {0.2, 0, 0}, std::nullopt, 1.0);
  EXPECT_EQ(d.state, State::kNoData);
  EXPECT_DOUBLE_EQ(d.out.vx, 0.2);
  d = g.update(0.1, {0.2, 0, 0}, 0.5, std::nullopt);
  EXPECT_EQ(d.state, State::kNoData);
  EXPECT_DOUBLE_EQ(d.out.vx, 0.2);
  g.update(1.0, {0.12, 0, 0}, 0.5, 0.1);  // latch
  d = g.update(1.5, {0.12, 0, 0}, std::nullopt, std::nullopt);  // sensors lost mid brake: keep braking
  EXPECT_DOUBLE_EQ(d.out.vx, 0.0);
  d = g.update(3.5, {0.12, 0, 0}, std::nullopt, std::nullopt);  // > max_brake_hold_s: fail-open to old behaviour
  EXPECT_DOUBLE_EQ(d.out.vx, 0.12);
  EXPECT_FALSE(g.latched());
}

TEST(Governor, PropertyNeverIncreasesNeverTouchesVyWz)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  ApproachGovernor g;
  double t = 0;
  for (int i = 0; i < 20000; ++i) {
    t += 0.02;
    Command c{u(rng) * 0.4, u(rng) * 0.2, u(rng)};
    std::optional<double> sp = (rng() % 5 == 0) ? std::nullopt : std::optional<double>(std::fabs(u(rng)));
    std::optional<double> rem = (rng() % 5 == 0) ? std::nullopt : std::optional<double>(std::fabs(u(rng)) * 3);
    auto d = g.update(t, c, sp, rem);
    ASSERT_DOUBLE_EQ(d.out.vy, c.vy);
    ASSERT_DOUBLE_EQ(d.out.wz, c.wz);
    ASSERT_TRUE(d.out.vx == c.vx || d.out.vx == 0.0);
    ASSERT_LE(std::fabs(d.out.vx), std::fabs(c.vx));
    ASSERT_EQ(d.modified, d.out.vx != c.vx);
  }
}

TEST(Governor, ResetClearsLatch)
{
  ApproachGovernor g;
  g.update(0.0, {0.12, 0, 0}, 0.5, 0.1);
  ASSERT_TRUE(g.latched());
  g.reset();
  EXPECT_FALSE(g.latched());
}

// REPLAY of recorded data (bag bag_e2e02_20261105_1707): the 8 clean stop events (cmd vx -> 0 while walking).
// v0 = LIO speed at the last non-zero command, d = LIO travel until the robot stopped (docs/results/20261105_d05_governor.md).
// With the governor the zero command would have been issued when remaining == stop_dist == stop_time * v0, so the
// stop point error is d - stop_time * v0. This uses only recorded motion, it does NOT simulate a new controller loop.
TEST(Replay, RecordedStopEventsLandWithinTolerance)
{
  struct Ev {double v0, d;};
  const Ev ev[] = {{0.47, 0.16}, {0.52, 0.27}, {0.44, 0.20}, {0.42, 0.07},
    {0.32, 0.17}, {0.78, 0.23}, {0.37, 0.18}, {0.78, 0.31}};
  Params p;
  double sum2 = 0.0, worst = 0.0;
  for (const auto & e : ev) {
    const double err = e.d - p.stop_time_s * e.v0;
    sum2 += err * err;
    worst = std::max(worst, std::fabs(err));
    EXPECT_LE(std::fabs(err), 0.10) << "v0=" << e.v0;
  }
  EXPECT_LT(std::sqrt(sum2 / 8.0), 0.07);
  EXPECT_LE(worst, 0.10);
  // Baseline for comparison: the old behaviour stops commanding at the goal tolerance (0.10 m): error = d - 0.10 .
  double base_worst = 0.0;
  for (const auto & e : ev) {base_worst = std::max(base_worst, std::fabs(e.d - 0.10));}
  EXPECT_GT(base_worst, 0.15);  // up to 0.21 m past the tolerance edge
}
