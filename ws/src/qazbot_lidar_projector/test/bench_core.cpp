// Copyright 2026 Qazbot
// Per-cloud timing of the core library on the fixture clouds (levelled with the fixture's own IMU window).
// Usage: bench_core FIXTURE [passes=20] [range_max=8.0]. Prints p50/p95/p99 [ms] and peak RSS [MB].
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include "fixture_reader.hpp"
#include "qazbot_lidar_projector/lidar_projection.hpp"

using namespace qazbot_lidar_projector;  // NOLINT

int main(int argc, char ** argv)
{
  if (argc < 2) {return 2;}
  const int passes = argc > 2 ? std::atoi(argv[2]) : 20;
  CoreParams p;
  p.range_max = argc > 3 ? std::atof(argv[3]) : 8.0;
  const auto events = fixture::read(argv[1]);
  GravityLeveller lev(p.level_s);
  PoseTracker trk(p.twist_baseline_s);
  struct Job {const fixture::Event * e; Pose pose;};
  std::vector<Job> jobs;
  for (const auto & e : events) {
    const std::int64_t key = static_cast<std::int64_t>(e.sec) * 1000000000LL + e.nanosec;
    const double t = e.sec + e.nanosec * 1e-9;
    if (e.kind == 0) {
      lev.addAccel(t, {e.v[0], e.v[1], e.v[2]});
    } else if (e.kind == 1) {
      const Mat3 r = quatToRot(e.v[3], e.v[4], e.v[5], e.v[6]);
      lev.setOrientation(r);
      if (lev.ready()) {trk.push(key, t, levelPose(lev.rotation(), r, {e.v[0], e.v[1], e.v[2]}, p.lidar_fwd_in_imu));}
    } else if (lev.ready() && trk.find(key)) {
      jobs.push_back({&e, *trk.find(key)});
    }
  }
  CloudProcessor proc(p);
  CloudResult res;
  std::vector<double> ms;
  for (int k = 0; k < passes; ++k) {
    for (const auto & j : jobs) {
      XyzView v;
      v.data = reinterpret_cast<const std::uint8_t *>(j.e->xyz.data());
      v.count = j.e->xyz.size() / 3; v.step = 12; v.off_x = 0; v.off_y = 4; v.off_z = 8;
      v.skip_nans = !j.e->dense;
      const auto t0 = std::chrono::steady_clock::now();
      proc.process(v, lev.rotation(), j.pose, res);
      ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    }
  }
  std::sort(ms.begin(), ms.end());
  auto q = [&](double f) {return ms[static_cast<std::size_t>(f * (ms.size() - 1))];};
  long rss_kb = 0;
  std::ifstream st("/proc/self/status");
  for (std::string l; std::getline(st, l);) {
    if (l.rfind("VmHWM:", 0) == 0) {rss_kb = std::atol(l.c_str() + 6);}
  }
  std::printf("clouds=%zu n=%zu p50_ms=%.4f p95_ms=%.4f p99_ms=%.4f peak_rss_mb=%.1f\n",
    jobs.size(), ms.size(), q(0.5), q(0.95), q(0.99), rss_kb / 1024.0);
  return 0;
}
