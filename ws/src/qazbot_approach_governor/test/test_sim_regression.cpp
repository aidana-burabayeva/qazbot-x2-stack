// SIMULATION (not a replay of recorded data). Closed loop: simplified RPP approach + X2 plant model + governor core.
//
// Plant model assumptions (each from measured data, docs/results/20261105_d05_governor.md):
//  A1 forward speed target = 0.50 m/s for any delayed forward command >= 0.10 m/s, 0 otherwise (measured: cmd 0.20 ->
//     0.50 +- 0.05, 0.30 -> 0.48, 0.12 within a moving gait -> ~0.5; steady response below 0.2 is NOT measured: gap)
//  A2 command dead time 0.40 s (cross-correlation peak 0.4-0.5 s), rise time constant 0.30 s
//  A3 after the command goes to zero the robot travels d = v * (0.40 + N(0, 0.12)) m until it stops (8 recorded stops:
//     median 0.38 s, residual sd 0.054 m), constant deceleration after 0.15 s
//  A4 an in-place turn-around (goal behind the robot, RPP rotate-to-heading) lasts 4-8 s and shifts the body by
//     N(0, 0.30) m along the path (E2E-02: 0.5-1.2 m net per turn)
//  A6 yaw error at arrival N(-0.12, 0.10) rad (E2E-02: -0.16..-0.27); Nav2 succeeds once xy (0.10 m band) and yaw (0.10 rad)
//     are both inside tolerance; the xy tolerance is a 0.10 m DISC
//  A7 lateral error at arrival N(0.12, 0.05) m (E2E-02 and SL0.20 runs: 0.09-0.18 m); RPP rotates in place toward the path end
//     when it is more than 0.6 rad off the heading (rotate_to_heading_min_angle) and the robot is outside the disc
//  A8 after such a turn the lateral error is |N(0, 0.06)| m (robot faces the carrot)
//  A5 pose noise 0.01 m (sd) on a 10 Hz pose stream -> speed noise ~0.05 m/s at the 0.3 s window (measured sd 0.055)
// RPP approach: v = 0.20 * min(1, remaining/0.6), floor 0.12 (min_approach), goal checker radius 0.10 m.
// The simulation compares baseline (no governor) with governor on identical random draws. It cannot prove behaviour
// on the robot; it checks that the logic does what it is designed for under these assumptions.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>

#include "qazbot_approach_governor/approach_governor.hpp"

using namespace qazbot_approach_governor;  // NOLINT

namespace
{
struct Result
{
  bool success{false};
  double time{0.0};
  double max_overshoot{0.0};   // beyond the goal along the path
  int turnarounds{0};
  double rotate_time{0.0};
};

struct Plant
{
  double x{0.0}, v{0.0};
  double stop_gain{0.40};        // d = v * stop_gain until stop
  std::deque<std::pair<double, double>> pipe;  // (release time, vx)
  double cmd_applied{0.0};
  bool braking{false};
  double decel{0.0}, brake_start{0.0};
  double v_target_walk{0.50}, v_start{0.10};
  std::mt19937 * rng{nullptr};

  void step(double t, double dt, double vx_cmd, double dead = 0.40)
  {
    pipe.emplace_back(t + dead, vx_cmd);
    while (!pipe.empty() && pipe.front().first <= t) {cmd_applied = pipe.front().second; pipe.pop_front();}
    const bool walk = cmd_applied >= v_start;
    if (walk) {
      braking = false;
      v += (v_target_walk - v) * (1.0 - std::exp(-dt / 0.30));
    } else {
      if (!braking && v > 0.05) {
        braking = true;
        brake_start = t;
        std::normal_distribution<double> n(0.0, 0.12);
        const double d_total = std::max(0.03 * v, v * std::max(0.1, stop_gain + n(*rng)));
        const double d_rem = std::max(0.02, d_total - v * 0.15);
        decel = v * v / (2.0 * d_rem);
      }
      if (braking && t - brake_start > 0.15) {v = std::max(0.0, v - decel * dt);}
      if (v < 0.02) {v = 0.0; braking = false;}
    }
    x += v * dt;
  }
};

Result run(bool use_governor, unsigned seed, double goal_dist = 3.0, double stop_time = 0.40, double lat_mean = 0.12)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> pose_noise(0.0, 0.01), unit(0.0, 1.0);
  Plant plant;
  plant.rng = &rng;
  Params p;
  p.stop_time_s = stop_time;
  ApproachGovernor gov(p);
  SpeedEstimator spd(p.speed_window_s, p.max_speed_age_s);
  Result r;
  const double dt = 0.02, T_END = 90.0;
  const double kBand = 0.10, kYawTol = 0.10, kRotRate = 0.5, kMinAngle = 0.6;
  double t = 0.0, next_pose = 0.0;
  double yaw_err = -0.12 + 0.10 * unit(rng);     // A6
  double y = std::max(0.0, lat_mean + 0.05 * unit(rng));  // A7: lateral error at arrival (E2E-02 and SL0.20 runs: 0.09-0.18 m)
  double rot_left = 0.0;
  bool turning = false;
  while (t < T_END) {
    if (t >= next_pose) {spd.push(t, plant.x + pose_noise(rng), 0.0); next_pose += 0.1;}
    const double rem = goal_dist - plant.x;  // along-track remaining, <0 beyond the goal
    r.max_overshoot = std::max(r.max_overshoot, -rem);
    const double d = std::hypot(rem, y);
    const double angle = std::fabs(std::atan2(y, rem));  // angle to the carrot (= path end when closer than the lookahead)
    double vx_cmd = 0.0;
    bool pure_rotation = false;
    if (turning) {
      pure_rotation = true;
      rot_left -= dt;
      if (rot_left <= 0.0) {turning = false; yaw_err = 0.10 * unit(rng); y = std::fabs(0.06 * unit(rng));}  // A8
    } else if (d <= kBand) {
      // RPP: inside the xy tolerance -> rotate to the goal heading (or done when yaw is inside tolerance as well)
      if (std::fabs(yaw_err) <= kYawTol) {r.success = true; r.time = t; return r;}
      pure_rotation = true;
      yaw_err -= std::copysign(kRotRate * dt, yaw_err);
    } else if (rem < 0.5 && angle > kMinAngle) {
      // RPP rotate-to-path: the carrot (path end) is more than 0.6 rad off the heading -> in-place turn toward it
      turning = true;
      ++r.turnarounds;
      rot_left = angle / 0.6;
      pure_rotation = true;
    } else {
      vx_cmd = std::max(0.12, 0.20 * std::min(1.0, std::max(rem, 0.0) / 0.6));
    }
    if (pure_rotation) {
      r.rotate_time += dt;
      plant.x += 0.13 * std::sqrt(dt) * unit(rng);  // A4: random-walk body drift during in-place turns
      y += 0.13 * std::sqrt(dt) * unit(rng);
      y = std::fabs(y);
    }
    if (use_governor) {
      vx_cmd = gov.update(t, {vx_cmd, 0.0, 0.0}, spd.speed(t), std::max(0.0, rem)).out.vx;
    }
    plant.step(t, dt, vx_cmd);
    t += dt;
  }
  r.success = false;
  r.time = T_END;
  return r;
}

struct Stats
{
  int n{0}, ok{0}, with_turn{0};
  double sum_turn{0}, sum_time{0}, worst_over{0}, sum_rot{0};
  std::vector<double> times;
};

Stats batch(bool gov, int n, double dist = 3.0, double stop_time = 0.40, double lat = 0.12)
{
  Stats s;
  for (int i = 0; i < n; ++i) {
    auto r = run(gov, 1000 + i, dist, stop_time, lat);
    ++s.n;
    s.ok += r.success;
    s.with_turn += r.turnarounds > 0;
    s.sum_turn += r.turnarounds;
    s.sum_rot += r.rotate_time;
    s.sum_time += r.time;
    s.times.push_back(r.time);
    s.worst_over = std::max(s.worst_over, r.max_overshoot);
  }
  std::sort(s.times.begin(), s.times.end());
  return s;
}

void report(const char * label, const Stats & b, const Stats & g)
{
  const char * out = std::getenv("SIM_OUT");
  FILE * f = out ? std::fopen(out, "a") : nullptr;
  auto line = [&](FILE * o) {
      std::fprintf(
        o, "%-28s | baseline: success %3d/%d, turn-arounds/run %.2f, rotation s/run %.1f, median t %.1f s, worst overshoot %.2f m"
        " | governor: success %3d/%d, turn-arounds/run %.2f, rotation s/run %.1f, median t %.1f s, worst overshoot %.2f m\n",
        label, b.ok, b.n, b.sum_turn / b.n, b.sum_rot / b.n, b.times[b.n / 2], b.worst_over, g.ok, g.n, g.sum_turn / g.n,
        g.sum_rot / g.n, g.times[g.n / 2], g.worst_over);
    };
  line(stdout);
  if (f) {line(f); std::fclose(f);}
}
}  // namespace

TEST(SimRegression, BaselineReproducesE2E02Symptom)
{
  // Without it the simulation says nothing about the fix: E2E-02 needed 60 s and 4 turn-arounds for one 3 m goal.
  auto b = batch(false, 200);
  EXPECT_GT(b.sum_turn / b.n, 3.0);
  EXPECT_GT(b.worst_over, 1.0);
  EXPECT_GT(b.times[b.n / 2], 10.0);
}

TEST(SimRegression, GovernorNeverWorseThanBaselineAtMeasuredLateralError)
{
  auto b = batch(false, 200), g = batch(true, 200);
  report("straight 3 m, lateral 0.12", b, g);
  EXPECT_GE(g.ok, b.ok);
  EXPECT_LE(g.sum_turn, b.sum_turn);
  EXPECT_LE(g.sum_rot, b.sum_rot);
}

TEST(SimRegression, LateralErrorDominatesTheResidual)
{
  // Sensitivity: the along-track governor helps most when the lateral error at arrival is inside the tolerance disc.
  for (double lat : {0.0, 0.04, 0.08}) {
    auto b = batch(false, 200, 3.0, 0.40, lat), g = batch(true, 200, 3.0, 0.40, lat);
    char name[40];
    std::snprintf(name, sizeof name, "straight 3 m, lateral %.2f", lat);
    report(name, b, g);
    EXPECT_GE(g.ok, b.ok - 4);           // never materially worse
    EXPECT_LE(g.sum_turn, 1.05 * b.sum_turn + 5);
  }
}

TEST(SimRegression, ShortGoals)
{
  for (double d : {0.8, 1.5}) {
    auto b = batch(false, 200, d), g = batch(true, 200, d);
    char name[40];
    std::snprintf(name, sizeof name, "straight %.1f m", d);
    report(name, b, g);
    EXPECT_LE(g.sum_turn, b.sum_turn);
    EXPECT_GE(g.ok, b.ok);
  }
}

TEST(SimRegression, StopTimeSensitivity)
{
  // Wrong stop_time must not make things worse than the baseline by a large margin (0.25 s: too short, 0.60 s: too long).
  auto b = batch(false, 200);
  for (double st : {0.25, 0.60}) {
    auto g = batch(true, 200, 3.0, st);
    char name[40];
    std::snprintf(name, sizeof name, "stop_time %.2f", st);
    report(name, b, g);
    EXPECT_LE(g.sum_turn, 1.2 * b.sum_turn);
  }
}

TEST(SimRegression, SpeedDataLostGovernorPassesThrough)
{
  // Odometry unavailable for the whole run: identical to the baseline (fail-open to the previous behaviour).
  std::mt19937 rng(5);
  Plant plant;
  plant.rng = &rng;
  ApproachGovernor gov;
  double t = 0, max_x = 0;
  for (int i = 0; i < 4000; ++i, t += 0.02) {
    const double rem = std::max(0.0, 3.0 - plant.x);
    const double vx = std::max(0.12, 0.20 * std::min(1.0, rem / 0.6));
    auto d = gov.update(t, {vx, 0, 0}, std::nullopt, rem);
    ASSERT_DOUBLE_EQ(d.out.vx, vx);
    plant.step(t, 0.02, d.out.vx);
    max_x = std::max(max_x, plant.x);
  }
  EXPECT_GT(max_x, 3.0);  // it simply keeps walking as before: no new restriction, no new safety claim
}

TEST(SimRegression, CancelAndRcTakeoverNeedNoGovernorAction)
{
  // Goal cancel / RC takeover: zero command or no command. A zero command is never modified, an existing latch only
  // keeps vx at zero (reduces motion), so the sequence zero -> RC cannot be blocked by the governor.
  ApproachGovernor gov;
  gov.update(0.0, {0.12, 0, 0}, 0.5, 0.1);  // latched
  auto d = gov.update(0.1, {0.0, 0.0, 0.0}, 0.4, 0.1);
  EXPECT_DOUBLE_EQ(d.out.vx, 0.0);
  EXPECT_DOUBLE_EQ(d.out.wz, 0.0);
  EXPECT_FALSE(d.modified);
}
