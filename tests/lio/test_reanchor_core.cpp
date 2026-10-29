#include "../../candidates/native_odom_reanchor/reanchor.hpp"
#include <cassert>
#include <iostream>
#include <limits>

using namespace qazbot::candidate;

int main()
{
  unsigned cases = 0;
  for (double jump : {0.467, 0.5, 1.563, 1.6}) {
    for (double gap : {0.1, 40.0}) {
      // Same source fault without reset evidence must latch permanently.
      Reanchor unexplained;
      assert(unexplained.update({100, {2, 3, 0.7}}, 100));
      assert(!unexplained.update({100 + gap, {2 + jump, 3, 0.7}}, 100 + gap));
      assert(unexplained.state() == State::latched);
      assert(!unexplained.motion_allowed(100 + gap, true));
      assert(!unexplained.begin_reset(1, true, true));
      ++cases;

      Reanchor reset;
      assert(reset.update({100, {2, 3, 0.7}}, 100));
      assert(!reset.begin_reset(1, true, false));
      assert(reset.begin_reset(1, true, true));
      assert(!reset.motion_allowed(100.01, true));
      const double t = 100 + gap;
      Sample raw{t, {2 + jump, 3, -0.4}}, anchor{t, {2, 3, 0.7}};
      assert(!reset.confirm_reset(1, raw, anchor, t, false, true, true));
      assert(!reset.confirm_reset(0, raw, anchor, t, true, true, true));
      assert(!reset.confirm_reset(1, raw, anchor, t + 1, true, true, true));
      assert(!reset.confirm_reset(1, raw, anchor, t, true, false, true));
      auto bad_anchor = anchor;
      bad_anchor.stamp += 0.01;
      assert(!reset.confirm_reset(1, raw, bad_anchor, t, true, true, true));
      assert(reset.confirm_reset(1, raw, anchor, t, true, true, true));
      assert(!reset.release(1, true, t));
      auto a = reset.update({t + .01, raw.pose}, t + .01);
      assert(a && std::hypot(a->pose.x - 2, a->pose.y - 3) < 1e-12);
      assert(std::abs(wrap(a->pose.yaw - .7)) < 1e-12);
      assert(!reset.motion_allowed(t + .01, true));
      Pose delta{.01, 0, .005};
      auto b = reset.update({t + .02, compose(raw.pose, delta)}, t + .02);
      auto expected = compose(anchor.pose, delta);
      assert(b && std::hypot(b->pose.x - expected.x, b->pose.y - expected.y) < 1e-12);
      assert(std::abs(wrap(b->pose.yaw - expected.yaw)) < 1e-12);
      assert(!reset.release(1, false, t + .02));
      assert(reset.release(1, true, t + .02));
      assert(reset.motion_allowed(t + .02, true));
      assert(!reset.motion_allowed(t + .02, false));
      assert(!reset.motion_allowed(t + 1, true));
      assert(!reset.release(1, true, t + .02));
      ++cases;
    }
  }
  Reanchor invalid;
  assert(!invalid.update({100, {std::numeric_limits<double>::quiet_NaN(), 0, 0}}, 100));
  assert(invalid.state() == State::latched);
  // Real displacement during a data gap must be retained by independent verification,
  // rather than silently freezing the robot at its pre-gap pose.
  Reanchor moved;
  assert(moved.update({100, {0, 0, 0}}, 100));
  assert(moved.begin_reset(1, true, true));
  assert(moved.confirm_reset(1, {140, {1.6, 0, .7}}, {140, {.15, -.05, .1}},
    140, true, true, true));
  auto recovered = moved.update({140.01, {1.6, 0, .7}}, 140.01);
  assert(recovered && std::abs(recovered->pose.x - .15) < 1e-12);
  assert(std::abs(recovered->pose.y + .05) < 1e-12);
  assert(!moved.update({140.02, {2.6, 0, .7}}, 140.02));
  assert(moved.state() == State::latched);
  assert(!moved.motion_allowed(140.02, true));
  std::cout << "{\"scenario_cases\":" << cases << ",\"result\":\"PASS\"}\n";
}
