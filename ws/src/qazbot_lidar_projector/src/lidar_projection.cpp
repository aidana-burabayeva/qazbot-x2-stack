// Copyright 2026 Qazbot
// Port of candidates/d04/level_bridge.py (D-04 variant C). Expression order follows the Python reference; this
// translation unit is compiled with -ffp-contract=off so that no fused multiply-add changes a rounding.
#include "qazbot_lidar_projector/lidar_projection.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace qazbot_lidar_projector
{

namespace
{
constexpr double kPi = 3.14159265358979323846;   // == Python math.pi
constexpr double kInf = std::numeric_limits<double>::infinity();

float readFloat(const std::uint8_t * p)
{
  float f;
  std::memcpy(&f, p, sizeof(f));
  return f;
}
}  // namespace

Mat3 identity3()
{
  return {1, 0, 0, 0, 1, 0, 0, 0, 1};
}

Mat3 matMul(const Mat3 & a, const Mat3 & b)
{
  Mat3 c{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      c[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
    }
  }
  return c;
}

Vec3 matVec(const Mat3 & a, const Vec3 & v)
{
  return {a[0] * v.x + a[1] * v.y + a[2] * v.z,
    a[3] * v.x + a[4] * v.y + a[5] * v.z,
    a[6] * v.x + a[7] * v.y + a[8] * v.z};
}

Mat3 quatToRot(double x, double y, double z, double w)
{
  return {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
}

Mat3 levelRotation(const Vec3 & up)
{
  const double n = std::sqrt(up.x * up.x + up.y * up.y + up.z * up.z);
  const Vec3 z{up.x / n, up.y / n, up.z / n};
  const Vec3 a = std::fabs(z.x) < 0.9 ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0};
  const double d = a.x * z.x + a.y * z.y + a.z * z.z;
  Vec3 x{a.x - d * z.x, a.y - d * z.y, a.z - d * z.z};
  const double xn = std::sqrt(x.x * x.x + x.y * x.y + x.z * x.z);
  x = {x.x / xn, x.y / xn, x.z / xn};
  const Vec3 y{z.y * x.z - z.z * x.y, z.z * x.x - z.x * x.z, z.x * x.y - z.y * x.x};
  return {x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z};
}

Mat6 rotateCovariance(const Mat3 & r, const Mat6 & cov)
{
  Mat6 b{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      b[6 * i + j] = r[3 * i + j];
      b[6 * (i + 3) + (j + 3)] = r[3 * i + j];
    }
  }
  Mat6 bs{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double s = 0.0;
      for (int k = 0; k < 6; ++k) {s += b[6 * i + k] * cov[6 * k + j];}
      bs[6 * i + j] = s;
    }
  }
  Mat6 out{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double s = 0.0;
      for (int k = 0; k < 6; ++k) {s += bs[6 * i + k] * b[6 * j + k];}   // (B^T)[k][j] = B[j][k]
      out[6 * i + j] = s;
    }
  }
  return out;
}

GravityLeveller::GravityLeveller(double level_s)
: level_s_(level_s) {}

void GravityLeveller::setOrientation(const Mat3 & r_wb)
{
  r_wb_ = r_wb;
  has_orientation_ = true;
}

bool GravityLeveller::addAccel(double stamp_s, const Vec3 & accel_body)
{
  if (ready_ || !has_orientation_) {
    return false;
  }
  // Python: `self.t_acc0 = self.t_acc0 or t` (a stamp of exactly 0.0 counts as unset).
  if (!has_t0_ || t0_ == 0.0) {
    t0_ = stamp_s;
    has_t0_ = true;
  }
  const Vec3 w = matVec(r_wb_, accel_body);
  sum_.x += w.x;
  sum_.y += w.y;
  sum_.z += w.z;
  ++count_;
  if (stamp_s - t0_ >= level_s_) {
    const double n = static_cast<double>(count_);
    const Vec3 up{sum_.x / n, sum_.y / n, sum_.z / n};
    r_lv_ = levelRotation(up);
    const double un = std::sqrt(up.x * up.x + up.y * up.y + up.z * up.z);
    up_ = {up.x / un, up.y / un, up.z / un};
    ready_ = true;
    return true;
  }
  return false;
}

Pose levelPose(const Mat3 & r_lv, const Mat3 & r_wb, const Vec3 & p_world, const Vec3 & lidar_fwd_in_imu)
{
  Pose out;
  out.p_lv = matVec(r_lv, p_world);
  const Vec3 f = matVec(matMul(r_lv, r_wb), lidar_fwd_in_imu);
  out.x = out.p_lv.x;
  out.y = out.p_lv.y;
  out.yaw = std::atan2(f.y, f.x);
  return out;
}

PoseTracker::PoseTracker(double twist_baseline_s, std::size_t max_poses)
: baseline_s_(twist_baseline_s), max_poses_(max_poses) {}

PoseTracker::Update PoseTracker::push(std::int64_t key_ns, double stamp_s, const Pose & pose)
{
  Update u;
  if (has_last_ && key_ns < last_key_) {
    u.clock_stepped_back = true;
    u.step_back_s = static_cast<double>(last_key_ - key_ns) * 1e-9;
    poses_.clear();
    track_.clear();
  }
  has_last_ = true;
  last_key_ = key_ns;
  poses_[key_ns] = pose;
  while (poses_.size() > max_poses_) {
    poses_.erase(poses_.begin());
  }
  track_.push_back({stamp_s, pose.x, pose.y, pose.yaw});
  while (track_.size() > 1 && stamp_s - track_[1].t >= baseline_s_) {
    track_.pop_front();
  }
  const Sample & s0 = track_.front();
  if (stamp_s - s0.t < baseline_s_) {
    return u;   // no velocity yet: no Odometry rather than a zero twist
  }
  const double dt = stamp_s - s0.t;
  const double dx = pose.x - s0.x, dy = pose.y - s0.y;
  const double c = std::cos(-pose.yaw), sn = std::sin(-pose.yaw);
  u.has_twist = true;
  u.twist.vx = (c * dx - sn * dy) / dt;
  u.twist.vy = (sn * dx + c * dy) / dt;
  u.twist.wz = std::atan2(std::sin(pose.yaw - s0.yaw), std::cos(pose.yaw - s0.yaw)) / dt;
  return u;
}

const Pose * PoseTracker::find(std::int64_t key_ns) const
{
  const auto it = poses_.find(key_ns);
  return it == poses_.end() ? nullptr : &it->second;
}

CloudPoseSync::CloudAction CloudPoseSync::onCloud(
  std::int64_t key, bool pose_known, bool has_odom, std::int64_t newest_key)
{
  if (has_pending_) {
    // A newer cloud arrived before the held one was matched: the held one is superseded.
    has_pending_ = false;
    ++counters_.dropped_pending;
  }
  if (pose_known) {
    ++counters_.processed_direct;
    return CloudAction::kProcess;
  }
  if (!has_odom || key > newest_key) {
    // Its odometry has not been delivered yet (inter-topic delivery order): hold it.
    has_pending_ = true;
    pending_key_ = key;
    ++counters_.deferred;
    return CloudAction::kDefer;
  }
  ++counters_.dropped_no_pose;
  return CloudAction::kDrop;
}

CloudPoseSync::OdomAction CloudPoseSync::onOdom(std::int64_t key)
{
  if (!has_pending_ || key < pending_key_) {
    return OdomAction::kNone;
  }
  has_pending_ = false;
  if (key == pending_key_) {
    ++counters_.released;
    return OdomAction::kRelease;
  }
  ++counters_.dropped_pending;
  return OdomAction::kDropPending;
}

void BodyPoints::clear()
{
  bx.clear(); by.clear(); r.clear(); a.clear(); z.clear(); flat.clear();
}

void BodyPoints::reserve(std::size_t n)
{
  bx.reserve(n); by.reserve(n); r.reserve(n); a.reserve(n); z.reserve(n); flat.reserve(n);
}

void BodyPoints::push(double qx, double qy, double qz, double c, double s)
{
  if (!(std::isfinite(qx) && std::isfinite(qy) && std::isfinite(qz))) {
    return;
  }
  const double x = c * qx - s * qy;
  const double y = s * qx + c * qy;
  bx.push_back(x);
  by.push_back(y);
  r.push_back(std::hypot(x, y));
  a.push_back(std::atan2(y, x));
  z.push_back(qz);
  flat.push_back(std::hypot(qx, qy));
}

int binIndex(double a, int n_bins)
{
  const auto k = static_cast<long long>((a + kPi) / (2 * kPi) * n_bins);
  return static_cast<int>(k % n_bins);
}

bool inBand(double z, double flat, const CoreParams & p)
{
  return (z > p.z_min + p.floor_margin_per_m * flat) && (z < p.z_max);
}

std::size_t prepareBodyPoints(const XyzView & cloud, const Mat3 & r_lv, const Pose & pose, BodyPoints & out)
{
  out.clear();
  out.reserve(cloud.count);
  const double c = std::cos(-pose.yaw), s = std::sin(-pose.yaw);
  std::size_t read = 0;
  for (std::size_t i = 0; i < cloud.count; ++i) {
    const std::uint8_t * p = cloud.data + i * cloud.step;
    const double x = readFloat(p + cloud.off_x);
    const double y = readFloat(p + cloud.off_y);
    const double z = readFloat(p + cloud.off_z);
    if (cloud.skip_nans && (std::isnan(x) || std::isnan(y) || std::isnan(z))) {
      continue;
    }
    ++read;
    const double qx = (x * r_lv[0] + y * r_lv[1] + z * r_lv[2]) - pose.p_lv.x;
    const double qy = (x * r_lv[3] + y * r_lv[4] + z * r_lv[5]) - pose.p_lv.y;
    const double qz = (x * r_lv[6] + y * r_lv[7] + z * r_lv[8]) - pose.p_lv.z;
    out.push(qx, qy, qz, c, s);
  }
  return read;
}

void scanFromBody(const BodyPoints & pts, const CoreParams & p, std::vector<float> & ranges,
  std::vector<float> & obstacles_xyz)
{
  const auto n = static_cast<std::size_t>(p.n_bins);
  thread_local std::vector<double> best_r;
  thread_local std::vector<double> best_a;
  best_r.assign(n, kInf);
  best_a.assign(n, 0.0);
  ranges.assign(n, std::numeric_limits<float>::infinity());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    if (!inBand(pts.z[i], pts.flat[i], p)) {
      continue;
    }
    const double r = pts.r[i];
    if (!(r > p.range_min && r < p.range_max)) {
      continue;
    }
    const auto idx = static_cast<std::size_t>(binIndex(pts.a[i], p.n_bins));
    ranges[idx] = std::min(ranges[idx], static_cast<float>(r));
    if (r < best_r[idx]) {   // strict: the first point wins a tie, as np.lexsort is stable
      best_r[idx] = r;
      best_a[idx] = pts.a[i];
    }
  }
  obstacles_xyz.clear();
  for (std::size_t b = 0; b < n; ++b) {
    if (best_r[b] < kInf) {
      obstacles_xyz.push_back(static_cast<float>(best_r[b] * std::cos(best_a[b])));
      obstacles_xyz.push_back(static_cast<float>(best_r[b] * std::sin(best_a[b])));
      obstacles_xyz.push_back(0.0f);
    }
  }
}

void clearingFromBody(const BodyPoints & pts, const CoreParams & p, std::vector<float> & clearing_xyz)
{
  const auto n = static_cast<std::size_t>(p.n_bins);
  thread_local std::vector<double> obstacle;
  thread_local std::vector<double> best_end;
  thread_local std::vector<long long> best_i;
  obstacle.assign(n, kInf);
  best_end.assign(n, 0.0);
  best_i.assign(n, -1);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const double r = pts.r[i];
    if (!(r > 0)) {
      continue;
    }
    const double z = pts.z[i];
    double t = std::min(1.0, p.range_max / r);
    // Along a ray z(t) = t z, flat(t) = t r: stop at the first intersection with
    // z = z_min + margin * flat or z = z_max.
    const double slope = z - p.floor_margin_per_m * r;
    if (slope < 0) {t = std::min(t, p.z_min / slope);}
    if (z > 0) {t = std::min(t, p.z_max / z);}
    const double end = std::nextafter(r * t, 0.0);
    const auto idx = static_cast<std::size_t>(binIndex(pts.a[i], p.n_bins));
    if ((r > p.range_min) && (r < p.range_max) && (z > p.z_min + p.floor_margin_per_m * r) && (z < p.z_max)) {
      obstacle[idx] = std::min(obstacle[idx], r);
    }
    if (best_i[idx] < 0 || end > best_end[idx]) {   // strict: the first point wins a tie
      best_end[idx] = end;
      best_i[idx] = static_cast<long long>(i);
    }
  }
  clearing_xyz.clear();
  for (std::size_t b = 0; b < n; ++b) {
    if (best_i[b] < 0) {
      continue;
    }
    const auto i = static_cast<std::size_t>(best_i[b]);
    const double bounded = std::nextafter(std::min(best_end[b], obstacle[b]), 0.0);
    if (!(bounded > p.range_min)) {
      continue;
    }
    const double scale = bounded / pts.r[i];
    clearing_xyz.push_back(static_cast<float>(pts.bx[i] * scale));
    clearing_xyz.push_back(static_cast<float>(pts.by[i] * scale));
    clearing_xyz.push_back(0.0f);
  }
}

CloudProcessor::CloudProcessor(const CoreParams & params)
: params_(params) {}

bool CloudProcessor::process(const XyzView & cloud, const Mat3 & r_lv, const Pose & pose, CloudResult & out)
{
  if (prepareBodyPoints(cloud, r_lv, pose, body_) == 0) {
    return false;
  }
  clearingFromBody(body_, params_, out.clearing_xyz);
  scanFromBody(body_, params_, out.ranges, out.obstacles_xyz);
  return true;
}

}  // namespace qazbot_lidar_projector
