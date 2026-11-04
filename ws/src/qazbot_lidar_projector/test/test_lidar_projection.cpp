// Copyright 2026 Qazbot
// Unit tests of the pure core (no ROS). Cases mirror the behaviour of candidates/d04/level_bridge.py.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "qazbot_lidar_projector/lidar_projection.hpp"

using namespace qazbot_lidar_projector;  // NOLINT

namespace
{
const double kInf = std::numeric_limits<double>::infinity();

BodyPoints bodyFrom(const std::vector<Vec3> & q)   // levelled points, yaw 0
{
  BodyPoints b;
  for (const auto & p : q) {b.push(p.x, p.y, p.z, 1.0, 0.0);}
  return b;
}

Vec3 polar(double r, double deg, double z)
{
  const double a = deg * M_PI / 180.0;
  return {r * std::cos(a), r * std::sin(a), z};
}
}  // namespace

TEST(Math, QuatToRotIdentityAndQuarterTurn)
{
  const Mat3 i = quatToRot(0, 0, 0, 1);
  for (int k = 0; k < 9; ++k) {EXPECT_DOUBLE_EQ(i[k], identity3()[k]);}
  const double h = std::sqrt(0.5);
  const Vec3 v = matVec(quatToRot(0, 0, h, h), {1, 0, 0});   // +90 deg about z
  EXPECT_NEAR(v.x, 0.0, 1e-15);
  EXPECT_NEAR(v.y, 1.0, 1e-15);
}

TEST(Math, MatMulAgainstIdentity)
{
  const Mat3 r = quatToRot(0.1, 0.2, 0.3, std::sqrt(1 - 0.14));
  const Mat3 p = matMul(r, identity3());
  for (int k = 0; k < 9; ++k) {EXPECT_DOUBLE_EQ(p[k], r[k]);}
}

TEST(Math, LevelRotationMapsUpToZAndIsOrthonormal)
{
  for (const Vec3 up : {Vec3{0.3, -0.2, 9.7}, Vec3{9.8, 0.1, 0.2}, Vec3{-9.8, 0.1, 0.2}, Vec3{0.1, 9.8, 0.1}}) {
    const Mat3 r = levelRotation(up);
    const double n = std::sqrt(up.x * up.x + up.y * up.y + up.z * up.z);
    const Vec3 u = matVec(r, up);
    EXPECT_NEAR(u.x, 0.0, 1e-14);
    EXPECT_NEAR(u.y, 0.0, 1e-14);
    EXPECT_NEAR(u.z, n, 1e-13);
    const Mat3 rrt = matMul(r, {r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]});
    for (int k = 0; k < 9; ++k) {EXPECT_NEAR(rrt[k], identity3()[k], 1e-14);}
  }
}

TEST(Math, RotateCovarianceIdentityAndRotationInvariants)
{
  Mat6 c{};
  for (int i = 0; i < 6; ++i) {c[7 * i] = 1.0 + i;}
  c[1] = c[6] = 0.25;
  const Mat6 same = rotateCovariance(identity3(), c);
  for (int k = 0; k < 36; ++k) {EXPECT_DOUBLE_EQ(same[k], c[k]);}
  const Mat6 rot = rotateCovariance(quatToRot(0, 0, std::sqrt(0.5), std::sqrt(0.5)), c);
  double tr = 0, tr0 = 0;
  for (int i = 0; i < 6; ++i) {tr += rot[7 * i]; tr0 += c[7 * i];}
  EXPECT_NEAR(tr, tr0, 1e-13);
  EXPECT_NEAR(rot[0], 2.0, 1e-14);    // x,y variances swapped by the quarter turn
  EXPECT_NEAR(rot[7], 1.0, 1e-14);
  EXPECT_NEAR(rot[1], -0.25, 1e-14);  // off-diagonal changes sign
}

TEST(Leveller, WaitsForOrientationThenAveragesLevelSeconds)
{
  GravityLeveller l(1.0);
  EXPECT_FALSE(l.addAccel(10.0, {0, 0, 9.8}));   // no odometry orientation yet: ignored
  l.setOrientation(identity3());
  EXPECT_TRUE(l.hasOrientation());
  EXPECT_FALSE(l.addAccel(10.0, {0.0, 0.0, 9.8}));
  EXPECT_FALSE(l.addAccel(10.5, {0.0, 0.0, 9.8}));
  EXPECT_FALSE(l.ready());
  EXPECT_TRUE(l.addAccel(11.0, {0.0, 0.0, 9.8}));
  EXPECT_TRUE(l.ready());
  EXPECT_NEAR(l.up().z, 1.0, 1e-15);
  const Mat3 r = l.rotation();
  EXPECT_NEAR(r[8], 1.0, 1e-15);
  EXPECT_FALSE(l.addAccel(12.0, {9.8, 0, 0}));   // done: later samples change nothing
  EXPECT_NEAR(l.up().z, 1.0, 1e-15);
}

TEST(Leveller, RotatesAccelerationIntoWorldAndTreatsZeroStampAsUnset)
{
  GravityLeveller l(1.0);
  const double h = std::sqrt(0.5);
  l.setOrientation(quatToRot(0, h, 0, h));   // +90 deg about y: body x -> world -z, body z -> world +x
  EXPECT_FALSE(l.addAccel(0.0, {0, 0, 9.8}));
  EXPECT_FALSE(l.addAccel(0.0, {0, 0, 9.8}));   // t0 == 0.0 is re-armed like Python's `t_acc0 or t`
  EXPECT_FALSE(l.addAccel(0.5, {0, 0, 9.8}));   // t0 still 0.0 => re-armed to 0.5
  EXPECT_FALSE(l.addAccel(1.0, {0, 0, 9.8}));
  EXPECT_TRUE(l.addAccel(1.5, {0, 0, 9.8}));
  EXPECT_NEAR(l.up().x, 1.0, 1e-15);
}

TEST(Pose, YawFromLidarForwardAxis)
{
  const Vec3 fwd{1, 0, 0};
  const Pose p = levelPose(identity3(), identity3(), {1, 2, 3}, fwd);
  EXPECT_DOUBLE_EQ(p.x, 1);
  EXPECT_DOUBLE_EQ(p.y, 2);
  EXPECT_DOUBLE_EQ(p.p_lv.z, 3);
  EXPECT_DOUBLE_EQ(p.yaw, 0.0);
  const double h = std::sqrt(0.5);
  EXPECT_NEAR(levelPose(identity3(), quatToRot(0, 0, h, h), {0, 0, 0}, fwd).yaw, M_PI / 2, 1e-15);
}

TEST(Tracker, CacheKeepsNewest50AndFindsByKey)
{
  PoseTracker t(0.3);
  for (int i = 0; i < 60; ++i) {t.push(1000 + i, i * 0.1, Pose{});}
  EXPECT_EQ(t.size(), 50u);
  EXPECT_EQ(t.find(1000), nullptr);
  EXPECT_NE(t.find(1010), nullptr);
  EXPECT_NE(t.find(1059), nullptr);
  EXPECT_EQ(t.find(5), nullptr);
}

TEST(Tracker, TwistNeedsBaselineAndIsInChildFrame)
{
  PoseTracker t(0.3);
  Pose p;
  p.yaw = M_PI / 2;   // heading +y: world +y motion is body +x
  for (int i = 0; i < 3; ++i) {
    p.y = 0.1 * i * 0.5;   // 0.5 m/s
    const auto u = t.push(i * 100000000LL, i * 0.1, p);
    EXPECT_FALSE(u.has_twist) << i;   // 0, 0.1, 0.2 s < 0.3 s baseline
  }
  p.y = 0.15;
  const auto u = t.push(300000000LL, 0.3, p);
  ASSERT_TRUE(u.has_twist);
  EXPECT_NEAR(u.twist.vx, 0.5, 1e-12);
  EXPECT_NEAR(u.twist.vy, 0.0, 1e-12);
  EXPECT_NEAR(u.twist.wz, 0.0, 1e-12);
}

TEST(Tracker, AngularVelocityWrapsAround)
{
  PoseTracker t(0.3);
  Pose p;
  p.yaw = M_PI - 0.05;
  t.push(0, 0.0, p);
  p.yaw = -M_PI + 0.05;   // +0.1 rad across the wrap
  const auto u = t.push(300000000LL, 0.3, p);
  ASSERT_TRUE(u.has_twist);
  EXPECT_NEAR(u.twist.wz, 0.1 / 0.3, 1e-12);
}

TEST(Tracker, ClockStepBackClearsCacheAndBaseline)
{
  PoseTracker t(0.3);
  t.push(5000000000LL, 5.0, Pose{});
  t.push(5100000000LL, 5.1, Pose{});
  const auto u = t.push(1000000000LL, 1.0, Pose{});
  EXPECT_TRUE(u.clock_stepped_back);
  EXPECT_NEAR(u.step_back_s, 4.1, 1e-9);
  EXPECT_FALSE(u.has_twist);
  EXPECT_EQ(t.size(), 1u);
  EXPECT_EQ(t.find(5000000000LL), nullptr);
  EXPECT_NE(t.find(1000000000LL), nullptr);
}

TEST(Bins, IndexEdgesAndWrap)
{
  EXPECT_EQ(binIndex(-M_PI, 1440), 0);
  EXPECT_EQ(binIndex(M_PI, 1440), 0);   // a == pi wraps to bin 0, as `% N_BINS`
  EXPECT_EQ(binIndex(0.0, 1440), 720);
  EXPECT_EQ(binIndex(-M_PI + 0.9 * 2 * M_PI / 1440, 1440), 0);
  EXPECT_EQ(binIndex(-M_PI + 1.1 * 2 * M_PI / 1440, 1440), 1);
}

TEST(Band, FloorMarginGrowsWithRange)
{
  CoreParams p;
  EXPECT_TRUE(inBand(0.0, 1.0, p));
  EXPECT_FALSE(inBand(-0.65, 0.0, p));                 // strict lower bound
  EXPECT_TRUE(inBand(-0.60, 0.0, p));
  EXPECT_FALSE(inBand(-0.60, 5.0, p));                 // -0.65 + 0.035*5 = -0.475
  EXPECT_FALSE(inBand(0.80, 1.0, p));                  // strict upper bound
  EXPECT_FALSE(inBand(std::nan(""), 1.0, p));
}

TEST(Prepare, StridedViewNanSkippingAndLevelling)
{
  struct P {float pad; float y; float x; float z; float tail[2];};   // padded, x/y/z out of order
  std::vector<P> pts = {{0, 2, 1, 0.5f, {0, 0}}, {0, 0, std::nanf(""), 0, {0, 0}}, {0, 1, 1, 0.25f, {0, 0}}};
  XyzView v;
  v.data = reinterpret_cast<const std::uint8_t *>(pts.data());
  v.count = pts.size();
  v.step = sizeof(P);
  v.off_x = offsetof(P, x); v.off_y = offsetof(P, y); v.off_z = offsetof(P, z);
  Pose pose;
  pose.p_lv = {0.5, 0, 0};
  BodyPoints b;
  v.skip_nans = true;
  EXPECT_EQ(prepareBodyPoints(v, identity3(), pose, b), 2u);
  ASSERT_EQ(b.size(), 2u);
  EXPECT_NEAR(b.bx[0], 0.5, 1e-7);   // x = ? offset: field order checked below
  v.skip_nans = false;               // dense cloud claiming no NaN: the NaN point is read but dropped as non-finite
  EXPECT_EQ(prepareBodyPoints(v, identity3(), pose, b), 3u);
  EXPECT_EQ(b.size(), 2u);
}

TEST(Prepare, YawRotatesIntoBody)
{
  float xyz[3] = {0.f, 2.f, 0.f};   // levelled +y
  XyzView v;
  v.data = reinterpret_cast<const std::uint8_t *>(xyz);
  v.count = 1; v.step = 12; v.off_x = 0; v.off_y = 4; v.off_z = 8;
  Pose pose;
  pose.yaw = M_PI / 2;   // body x == levelled +y
  BodyPoints b;
  prepareBodyPoints(v, identity3(), pose, b);
  EXPECT_NEAR(b.bx[0], 2.0, 1e-15);
  EXPECT_NEAR(b.by[0], 0.0, 1e-15);
  EXPECT_NEAR(b.r[0], 2.0, 1e-15);
}

TEST(Scan, NearestInBandPerBinWithRangeLimits)
{
  CoreParams p;
  p.range_max = 8.0;
  const auto b = bodyFrom({
    polar(3.0, 10, 0.0), polar(2.0, 10.01, 0.0),    // same bin: nearest wins
    polar(0.45, 30, 0.0), polar(0.30, 40, 0.0),     // r == range_min and r < range_min: both dropped
    polar(8.0, 50, 0.0), polar(9.0, 60, 0.0),       // r == range_max and beyond: dropped
    polar(2.0, 70, 1.0),                            // above z_max
    polar(2.0, 80, -0.9),                           // floor
    Vec3{std::nan(""), 0, 0}});
  std::vector<float> ranges, obs;
  scanFromBody(b, p, ranges, obs);
  ASSERT_EQ(ranges.size(), 1440u);
  int finite = 0;
  for (float r : ranges) {finite += std::isfinite(r);}
  EXPECT_EQ(finite, 1);
  EXPECT_NEAR(ranges[binIndex(10.01 * M_PI / 180, 1440)], 2.0f, 1e-6);
  ASSERT_EQ(obs.size(), 3u);
  EXPECT_NEAR(obs[0], 2.0 * std::cos(10.01 * M_PI / 180), 1e-6);
  EXPECT_NEAR(obs[1], 2.0 * std::sin(10.01 * M_PI / 180), 1e-6);
  EXPECT_EQ(obs[2], 0.0f);
}

TEST(Scan, ObstaclesInBinOrderAndEmptyScan)
{
  CoreParams p;
  std::vector<float> ranges, obs;
  scanFromBody(bodyFrom({}), p, ranges, obs);
  EXPECT_TRUE(obs.empty());
  EXPECT_EQ(ranges.size(), 1440u);
  scanFromBody(bodyFrom({polar(2, 90, 0), polar(2, -90, 0), polar(2, 0, 0)}), p, ranges, obs);
  ASSERT_EQ(obs.size(), 9u);
  EXPECT_LT(obs[1], 0);      // -90 first
  EXPECT_NEAR(obs[4], 0, 1e-6);
  EXPECT_GT(obs[7], 0);
}

TEST(Clearing, FloorReturnEndsAtBandExitAndNoPointNoRay)
{
  CoreParams p;
  p.range_max = 8.0;
  // A floor return 1 m ahead, 1 m below: ray leaves the band at z = -0.65 + 0.035*flat.
  const auto b = bodyFrom({{1.0, 0.0, -1.0}});
  std::vector<float> c;
  clearingFromBody(b, p, c);
  ASSERT_EQ(c.size(), 3u);
  // z(t) = -t, flat(t) = t  =>  -t = -0.65 + 0.035 t  =>  t = 0.65 / 1.035
  EXPECT_NEAR(c[0], 0.65 / 1.035, 1e-6);
  EXPECT_NEAR(c[1], 0.0, 1e-9);
  EXPECT_EQ(c[2], 0.0f);
  clearingFromBody(bodyFrom({}), p, c);
  EXPECT_TRUE(c.empty());
}

TEST(Clearing, FarPointStopsJustShortOfRangeMaxAndCeilingExit)
{
  CoreParams p;
  p.range_max = 8.0;
  std::vector<float> c;
  clearingFromBody(bodyFrom({{20.0, 0.0, 0.0}}), p, c);   // level far point: capped by range_max
  ASSERT_EQ(c.size(), 3u);
  EXPECT_LE(c[0], 8.0f);   // nextafter(8, 0) rounds back to 8.0f in float32
  EXPECT_GT(c[0], 7.999f);
  clearingFromBody(bodyFrom({{2.0, 0.0, 1.6}}), p, c);    // above the ceiling: exits at z_max (t = 0.5)
  ASSERT_EQ(c.size(), 3u);
  EXPECT_NEAR(c[0], 1.0, 1e-6);
}

TEST(Clearing, CappedJustShortOfNearestInBandHitInSameBin)
{
  CoreParams p;
  p.range_max = 8.0;
  // Same bin: obstacle at 2 m (in band) and a far in-band point at 6 m: ray reaches 6 m but is capped below 2 m.
  std::vector<float> c;
  clearingFromBody(bodyFrom({{2.0, 0.0, 0.0}, {6.0, 0.0, 0.0}}), p, c);
  ASSERT_EQ(c.size(), 3u);
  EXPECT_LE(c[0], 2.0f);
  EXPECT_GT(c[0], 1.9999f);
  // Only a floor return in the bin: its short free ray is the result, a far return in another bin does not extend it.
  clearingFromBody(bodyFrom({{1.0, 0.0, -1.0}, polar(6.0, 20, 0.0)}), p, c);
  ASSERT_EQ(c.size(), 6u);
  EXPECT_NEAR(c[0], 0.65 / 1.035, 1e-6);
}

TEST(Clearing, LongestRayWinsAndRaysUnderRangeMinAreDropped)
{
  CoreParams p;
  p.range_max = 8.0;
  std::vector<float> c;
  // Floor ray (0.63 m) vs far in-band-exit ray in the same bin: the longer observed ray wins, capped by nothing.
  clearingFromBody(bodyFrom({{1.0, 0.0, -1.0}, {5.0, 0.0, -0.3}}), p, c);
  ASSERT_EQ(c.size(), 3u);
  EXPECT_GT(c[0], 4.0f);
  // Point at 0.3 m in band: capped below 0.3 < range_min => no ray.
  clearingFromBody(bodyFrom({{0.3, 0.0, 0.0}}), p, c);
  EXPECT_TRUE(c.empty());
  // A zero-range point is not a ray.
  clearingFromBody(bodyFrom({{0.0, 0.0, 0.0}}), p, c);
  EXPECT_TRUE(c.empty());
}

TEST(Clearing, TieKeepsFirstPoint)
{
  CoreParams p;
  p.range_max = 8.0;
  std::vector<float> c;
  // Two points in one bin with the same range-max-capped end: the first one's bearing is used (stable lexsort).
  const Vec3 a = polar(10.0, 10.0, 0.0), b = polar(10.0, 10.05, 0.0);
  ASSERT_EQ(binIndex(std::atan2(a.y, a.x), 1440), binIndex(std::atan2(b.y, b.x), 1440));
  clearingFromBody(bodyFrom({a, b}), p, c);
  ASSERT_EQ(c.size(), 3u);
  EXPECT_NEAR(std::atan2(c[1], c[0]), 10.0 * M_PI / 180, 1e-6);
}

TEST(Processor, EndToEndAndEmptyCloud)
{
  CoreParams p;
  p.range_max = 8.0;
  CloudProcessor proc(p);
  CloudResult out;
  XyzView empty;
  EXPECT_FALSE(proc.process(empty, identity3(), Pose{}, out));
  float xyz[6] = {2.f, 0.f, 0.f, 1.f, 0.f, -1.f};
  XyzView v;
  v.data = reinterpret_cast<const std::uint8_t *>(xyz);
  v.count = 2; v.step = 12; v.off_x = 0; v.off_y = 4; v.off_z = 8;
  ASSERT_TRUE(proc.process(v, identity3(), Pose{}, out));
  EXPECT_EQ(out.ranges.size(), 1440u);
  EXPECT_NEAR(out.ranges[720], 2.0f, 1e-6);
  ASSERT_EQ(out.obstacles_xyz.size(), 3u);
  ASSERT_EQ(out.clearing_xyz.size(), 3u);
  EXPECT_LE(out.clearing_xyz[0], 2.0f);
  EXPECT_DOUBLE_EQ(proc.params().range_max, 8.0);
  // all-NaN non-dense cloud: nothing to publish
  float nn[3] = {std::nanf(""), 0.f, 0.f};
  v.data = reinterpret_cast<const std::uint8_t *>(nn);
  v.count = 1; v.skip_nans = true;
  EXPECT_FALSE(proc.process(v, identity3(), Pose{}, out));
}

// ---- CloudPoseSync: cloud/odometry delivery order (root cause of the sporadic missing scans) ----
TEST(Sync, PoseKnownIsProcessedDirectly)
{
  CloudPoseSync s;
  EXPECT_EQ(s.onCloud(100, true, true, 100), CloudPoseSync::CloudAction::kProcess);
  EXPECT_FALSE(s.hasPending());
  EXPECT_EQ(s.onOdom(200), CloudPoseSync::OdomAction::kNone);
  EXPECT_EQ(s.counters().processed_direct, 1u);
}

TEST(Sync, CloudBeforeItsOdometryIsHeldAndReleasedBySameStamp)
{
  CloudPoseSync s;
  // newest odometry 100, cloud 200 arrives first: held, not dropped (old code returned silently here).
  EXPECT_EQ(s.onCloud(200, false, true, 100), CloudPoseSync::CloudAction::kDefer);
  EXPECT_TRUE(s.hasPending());
  EXPECT_EQ(s.pendingKey(), 200);
  EXPECT_EQ(s.onOdom(200), CloudPoseSync::OdomAction::kRelease);
  EXPECT_FALSE(s.hasPending());
  EXPECT_EQ(s.counters().deferred, 1u);
  EXPECT_EQ(s.counters().released, 1u);
  EXPECT_EQ(s.counters().dropped_pending + s.counters().dropped_no_pose, 0u);
}

TEST(Sync, CloudBeforeAnyOdometryIsHeld)
{
  CloudPoseSync s;
  EXPECT_EQ(s.onCloud(5, false, false, 0), CloudPoseSync::CloudAction::kDefer);
  EXPECT_EQ(s.onOdom(5), CloudPoseSync::OdomAction::kRelease);
}

TEST(Sync, OlderOdometryKeepsWaitingNewerOdometryDropsPending)
{
  CloudPoseSync s;
  EXPECT_EQ(s.onCloud(300, false, true, 100), CloudPoseSync::CloudAction::kDefer);
  EXPECT_EQ(s.onOdom(200), CloudPoseSync::OdomAction::kNone);   // older stamp: still waiting
  EXPECT_TRUE(s.hasPending());
  EXPECT_EQ(s.onOdom(400), CloudPoseSync::OdomAction::kDropPending);  // 300 was skipped by the odometry
  EXPECT_FALSE(s.hasPending());
  EXPECT_EQ(s.counters().dropped_pending, 1u);
}

TEST(Sync, MissingPoseOlderThanNewestOdometryIsDropped)
{
  CloudPoseSync s;
  EXPECT_EQ(s.onCloud(100, false, true, 200), CloudPoseSync::CloudAction::kDrop);
  EXPECT_FALSE(s.hasPending());
  EXPECT_EQ(s.counters().dropped_no_pose, 1u);
}

TEST(Sync, AtMostOneHeldCloudNewerCloudSupersedes)
{
  CloudPoseSync s;
  EXPECT_EQ(s.onCloud(200, false, true, 100), CloudPoseSync::CloudAction::kDefer);
  EXPECT_EQ(s.onCloud(300, false, true, 100), CloudPoseSync::CloudAction::kDefer);
  EXPECT_EQ(s.pendingKey(), 300);
  EXPECT_EQ(s.counters().dropped_pending, 1u);
  EXPECT_EQ(s.onCloud(400, true, true, 400), CloudPoseSync::CloudAction::kProcess);
  EXPECT_FALSE(s.hasPending());
  EXPECT_EQ(s.counters().dropped_pending, 2u);
}

TEST(Sync, TrackerExposesNewestKeyForTheSync)
{
  PoseTracker t(0.3);
  EXPECT_FALSE(t.hasNewest());
  t.push(1000, 1e-6, Pose{});
  t.push(2000, 2e-6, Pose{});
  EXPECT_TRUE(t.hasNewest());
  EXPECT_EQ(t.newestKey(), 2000);
}

// Replays an interleaving like the live one: odometry k then cloud k, except some stamps where the cloud is delivered
// first. With the sync no processed-parity cloud is lost; the old exact-lookup-or-return logic loses exactly those.
TEST(Sync, InterleavedReplayLosesNothing)
{
  PoseTracker t(0.3);
  CloudPoseSync s;
  int processed = 0, old_logic = 0;
  const std::vector<int> late_odom = {3, 17, 18, 40, 77};
  for (int k = 1; k <= 100; ++k) {
    const std::int64_t key = k * 100000000LL;
    const bool late = std::find(late_odom.begin(), late_odom.end(), k) != late_odom.end();
    auto cloud = [&]() {
        old_logic += t.find(key) != nullptr;
        if (s.onCloud(key, t.find(key) != nullptr, t.hasNewest(), t.newestKey()) ==
          CloudPoseSync::CloudAction::kProcess) {++processed;}
      };
    auto odom = [&]() {
        t.push(key, k * 0.1, Pose{});
        if (s.onOdom(key) == CloudPoseSync::OdomAction::kRelease) {++processed;}
      };
    if (late) {cloud(); odom();} else {odom(); cloud();}
  }
  EXPECT_EQ(processed, 100);
  EXPECT_EQ(old_logic, 100 - static_cast<int>(late_odom.size()));
  EXPECT_EQ(s.counters().released, late_odom.size());
}
