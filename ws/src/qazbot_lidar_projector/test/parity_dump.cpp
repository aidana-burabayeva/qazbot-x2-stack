// Copyright 2026 Qazbot
// Offline helper of test_parity.py: replays the fixture through the core library (levelling, pose tracker, cloud
// processor) and dumps every output. Usage: parity_dump FIXTURE OUT [range_max=8.0] [scan_every=1] [first_cloud_key_ns=0]
// (clouds with a stamp below first_cloud_key_ns are neither counted nor processed)
// Output (little-endian): 'R' + 9 f64 R_lv; per twist-bearing odometry 'O' + i64 key + x y z yaw (f64) + vx vy wz
// (f64) + 36 f64 covariance; per processed cloud 'C' + i64 key + 3 x (u32 count + f32 data): ranges, obstacles
// (xyz), clearing (xyz).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "fixture_reader.hpp"
#include "qazbot_lidar_projector/lidar_projection.hpp"

using namespace qazbot_lidar_projector;  // NOLINT

namespace
{
void put(std::FILE * f, const void * p, std::size_t n) {std::fwrite(p, 1, n, f);}
void putVec(std::FILE * f, const std::vector<float> & v)
{
  const std::uint32_t n = static_cast<std::uint32_t>(v.size());
  put(f, &n, 4);
  put(f, v.data(), v.size() * 4);
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {return 2;}
  CoreParams p;
  p.range_max = argc > 3 ? std::atof(argv[3]) : 8.0;
  const int every = argc > 4 ? std::atoi(argv[4]) : 1;
  const std::int64_t first_key = argc > 5 ? std::atoll(argv[5]) : 0;
  const auto events = fixture::read(argv[1]);
  std::FILE * out = std::fopen(argv[2], "wb");
  GravityLeveller lev(p.level_s);
  PoseTracker trk(p.twist_baseline_s);
  CloudProcessor proc(p);
  CloudResult res;
  int n_cloud = 0;
  for (const auto & e : events) {
    const std::int64_t key = static_cast<std::int64_t>(e.sec) * 1000000000LL + e.nanosec;
    const double t = e.sec + e.nanosec * 1e-9;
    if (e.kind == 0) {
      if (lev.addAccel(t, {e.v[0], e.v[1], e.v[2]})) {
        put(out, "R", 1);
        put(out, lev.rotation().data(), 72);
      }
    } else if (e.kind == 1) {
      const Mat3 r_wb = quatToRot(e.v[3], e.v[4], e.v[5], e.v[6]);
      lev.setOrientation(r_wb);
      if (!lev.ready()) {continue;}
      const Pose pose = levelPose(lev.rotation(), r_wb, {e.v[0], e.v[1], e.v[2]}, p.lidar_fwd_in_imu);
      const auto u = trk.push(key, t, pose);
      if (!u.has_twist) {continue;}
      Mat6 cov;
      std::memcpy(cov.data(), e.v + 7, 36 * 8);
      const Mat6 rc = rotateCovariance(lev.rotation(), cov);
      const double d[7] = {pose.x, pose.y, pose.p_lv.z, pose.yaw, u.twist.vx, u.twist.vy, u.twist.wz};
      put(out, "O", 1); put(out, &key, 8); put(out, d, sizeof(d)); put(out, rc.data(), 36 * 8);
    } else {
      if (key < first_key) {continue;}
      if (++n_cloud % every) {continue;}
      const Pose * pose = trk.find(key);
      if (pose == nullptr || !lev.ready()) {continue;}
      XyzView v;
      v.data = reinterpret_cast<const std::uint8_t *>(e.xyz.data());
      v.count = e.xyz.size() / 3; v.step = 12; v.off_x = 0; v.off_y = 4; v.off_z = 8;
      v.skip_nans = !e.dense;
      if (!proc.process(v, lev.rotation(), *pose, res)) {continue;}
      put(out, "C", 1); put(out, &key, 8);
      putVec(out, res.ranges); putVec(out, res.obstacles_xyz); putVec(out, res.clearing_xyz);
    }
  }
  std::fclose(out);
  return 0;
}
