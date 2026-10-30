// L1: fast kinematic MODEL of navigation on main_office (NOT Nav2, NOT MuJoCo). Simulation only.
// Per scenario: 8-connected A* on a grid inflated by the footprint half-width -> line-of-sight smoothing -> unicycle path
// following with rotate-in-place at both ends -> swept footprint clearance against occupied OR unknown cells.
// Output TSV: id category expected result reason t_s path_m min_clear_m. unknown is an obstacle, never free.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

struct Grid {
  int w = 0, h = 0; double res = 0.05, ox = 0, oy = 0;
  std::vector<unsigned char> free_;
  std::vector<float> edt;  // metres to the nearest non-free cell
};
static const double kFp[8][2] = {{0.17, 0.229}, {0.008, 0.34}, {-0.182, 0.34}, {-0.305, 0.229}, {-0.305, -0.261}, {-0.182, -0.372}, {0.008, -0.372}, {0.17, -0.261}};
static const double kHalfW = 0.356, kWmax = 0.5, kDt = 0.05;
static double kVmax = 0.4, kMargin = 0.03;  // MODEL parameters, overridable from argv for the config tournament  // v/w limits: MODEL assumptions, w from P20 measurement

static void dt1d(const float* f, float* d, int n, std::vector<int>& v, std::vector<float>& z) {
  int k = 0; v[0] = 0; z[0] = -1e20f; z[1] = 1e20f;
  for (int q = 1; q < n; ++q) {
    float s;
    while (true) { s = ((f[q] + q * (float)q) - (f[v[k]] + v[k] * (float)v[k])) / (2.f * q - 2.f * v[k]); if (s <= z[k] && k > 0) --k; else break; }
    if (s <= z[k]) { v[0] = q; z[0] = -1e20f; z[1] = 1e20f; k = 0; } else { ++k; v[k] = q; z[k] = s; z[k + 1] = 1e20f; }
  }
  k = 0;
  for (int q = 0; q < n; ++q) { while (z[k + 1] < q) ++k; d[q] = (q - v[k]) * (float)(q - v[k]) + f[v[k]]; }
}
static void edt(Grid& g) {
  const float INF = 1e12f; std::vector<float> f(g.w * g.h);
  for (size_t i = 0; i < f.size(); ++i) f[i] = g.free_[i] ? INF : 0.f;
  int m = std::max(g.w, g.h); std::vector<int> v(m); std::vector<float> z(m + 1), col(m), out(m);
  for (int x = 0; x < g.w; ++x) { for (int y = 0; y < g.h; ++y) col[y] = f[y * g.w + x]; dt1d(col.data(), out.data(), g.h, v, z); for (int y = 0; y < g.h; ++y) f[y * g.w + x] = out[y]; }
  for (int y = 0; y < g.h; ++y) { dt1d(&f[y * g.w], out.data(), g.w, v, z); for (int x = 0; x < g.w; ++x) f[y * g.w + x] = out[x]; }
  g.edt.resize(f.size()); for (size_t i = 0; i < f.size(); ++i) g.edt[i] = std::sqrt(f[i]) * (float)g.res;
}
static bool load(const std::string& dir, Grid& g) {
  std::ifstream y(dir + "/map.yaml"); std::string ln; double fr = 0.196, ot = 0.65;
  while (std::getline(y, ln)) {
    if (ln.rfind("resolution:", 0) == 0) g.res = atof(ln.c_str() + 11);
    if (ln.rfind("free_thresh:", 0) == 0) fr = atof(ln.c_str() + 12);
    if (ln.rfind("occupied_thresh:", 0) == 0) ot = atof(ln.c_str() + 16);
    if (ln == "origin:") { std::getline(y, ln); g.ox = atof(ln.c_str() + 2); std::getline(y, ln); g.oy = atof(ln.c_str() + 2); }
  }
  (void)ot;
  std::ifstream p(dir + "/map.pgm", std::ios::binary); std::string magic; int mx;
  p >> magic >> g.w >> g.h >> mx; p.get(); if (magic != "P5") return false;
  std::vector<unsigned char> px((size_t)g.w * g.h); p.read((char*)px.data(), px.size());
  g.free_.resize(px.size()); for (size_t i = 0; i < px.size(); ++i) g.free_[i] = ((255 - px[i]) / 255.0) < fr;
  edt(g); return true;
}
static inline void w2c(const Grid& g, double x, double y, int& c, int& r) { c = (int)std::floor((x - g.ox) / g.res); r = g.h - 1 - (int)std::floor((y - g.oy) / g.res); }
static inline double clearAt(const Grid& g, double x, double y) { int c, r; w2c(g, x, y, c, r); if (c < 0 || r < 0 || c >= g.w || r >= g.h) return -1; return g.edt[(size_t)r * g.w + c] - 0.5 * g.res; }  // distance to the nearest non-free cell EDGE; a point inside a non-free cell is < 0
// min clearance of the footprint outline (vertices + 4 samples per edge) at a pose
static double fpClear(const Grid& g, double x, double y, double th) {
  double mn = 1e9, c = std::cos(th), s = std::sin(th);
  for (int i = 0; i < 8; ++i) for (int k = 0; k < 5; ++k) {
    double a = kFp[i][0] + (kFp[(i + 1) % 8][0] - kFp[i][0]) * k / 5.0, b = kFp[i][1] + (kFp[(i + 1) % 8][1] - kFp[i][1]) * k / 5.0;
    mn = std::min(mn, clearAt(g, x + c * a - s * b, y + s * a + c * b));
  }
  return mn;
}
struct Res { std::string result, reason; double t = 0, len = 0, minc = 1e9; };
static bool los(const Grid& g, double inf, double x0, double y0, double x1, double y1) {
  double d = std::hypot(x1 - x0, y1 - y0); int n = std::max(1, (int)(d / 0.05));
  for (int i = 0; i <= n; ++i) if (clearAt(g, x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) < kHalfW) return false;
  return true;
}
static Res run1(const Grid& g, double inf, double sx, double sy, double sth, double gx, double gy, double gth) {
  Res R; auto bad = [&](const char* why) { R.result = "REJECT"; R.reason = why; return R; };
  if (clearAt(g, gx, gy) < inf) return bad("goal_not_free_with_margin");
  if (clearAt(g, sx, sy) < inf) return bad("start_not_free_with_margin");
  int sc, sr, gc, gr; w2c(g, sx, sy, sc, sr); w2c(g, gx, gy, gc, gr);
  std::vector<float> cost((size_t)g.w * g.h, 1e30f); std::vector<int> par((size_t)g.w * g.h, -1);
  typedef std::pair<float, int> Q; std::priority_queue<Q, std::vector<Q>, std::greater<Q>> pq;
  auto id = [&](int c, int r) { return r * g.w + c; };
  cost[id(sc, sr)] = 0; pq.push({0, id(sc, sr)});
  bool found = false;
  while (!pq.empty()) {
    auto [f, u] = pq.top(); pq.pop(); int uc = u % g.w, ur = u / g.w;
    if (uc == gc && ur == gr) { found = true; break; }
    if (f - std::hypot(uc - gc, ur - gr) > cost[u] + 1e-3) continue;
    for (int dr = -1; dr <= 1; ++dr) for (int dc = -1; dc <= 1; ++dc) {
      if (!dr && !dc) continue; int nc = uc + dc, nr = ur + dr; if (nc < 0 || nr < 0 || nc >= g.w || nr >= g.h) continue;
      if (g.edt[id(nc, nr)] - 0.5f * (float)g.res < inf || !g.free_[id(nc, nr)]) continue;
      // cheap clearance preference keeps the path off walls (centreline-ish)
      float step = (dr && dc ? 1.414f : 1.f) * (1.f + 0.5f * std::max(0.f, 1.0f - g.edt[id(nc, nr)]));
      float nco = cost[u] + step; if (nco < cost[id(nc, nr)]) { cost[id(nc, nr)] = nco; par[id(nc, nr)] = u; pq.push({nco + (float)std::hypot(nc - gc, nr - gr), id(nc, nr)}); }
    }
  }
  if (!found) return bad("no_geometric_path");
  std::vector<std::pair<double, double>> raw;
  for (int u = id(gc, gr); u != -1; u = par[u]) raw.push_back({g.ox + (u % g.w + 0.5) * g.res, g.oy + (g.h - (u / g.w) - 0.5) * g.res});
  std::reverse(raw.begin(), raw.end());
  std::vector<std::pair<double, double>> path{raw[0]}; size_t i = 0;
  while (i + 1 < raw.size()) { size_t j = raw.size() - 1; while (j > i + 1 && !los(g, inf, raw[i].first, raw[i].second, raw[j].first, raw[j].second)) --j; path.push_back(raw[j]); i = j; }
  // unicycle follower: rotate to first segment, drive with look-ahead on waypoints, rotate to goal yaw
  double x = sx, y = sy, th = sth, t = 0; auto upd = [&]() { R.minc = std::min(R.minc, fpClear(g, x, y, th)); };
  auto rot = [&](double target) { int guard = 0; while (std::fabs(std::remainder(target - th, 2 * M_PI)) > 0.05 && guard++ < 4000) { double e = std::remainder(target - th, 2 * M_PI); double w = std::max(-kWmax, std::min(kWmax, 2.0 * e)); if (std::fabs(w) < 0.05) w = w < 0 ? -0.05 : 0.05; th += w * kDt; t += kDt; upd(); } };
  upd();
  for (size_t k = 1; k < path.size(); ++k) {
    rot(std::atan2(path[k].second - y, path[k].first - x)); int guard = 0;
    while (std::hypot(path[k].first - x, path[k].second - y) > 0.05 && guard++ < 40000) {
      double d = std::hypot(path[k].first - x, path[k].second - y), hd = std::atan2(path[k].second - y, path[k].first - x), e = std::remainder(hd - th, 2 * M_PI);
      double v = std::min(kVmax, std::max(0.05, d)), w = std::max(-kWmax, std::min(kWmax, 2.0 * e));
      x += v * std::cos(th) * kDt; y += v * std::sin(th) * kDt; th += w * kDt; t += kDt; R.len += v * kDt; upd();
    }
  }
  rot(gth);
  R.t = t; double xyerr = std::hypot(x - gx, y - gy);
  if (R.minc < 0) { R.result = "COLLISION"; R.reason = "footprint_touches_obstacle_or_unknown"; }
  else if (xyerr > 0.2) { R.result = "FAIL"; R.reason = "xy_error"; }
  else { R.result = "REACHED"; R.reason = ""; }
  return R;
}
// strict first (circumscribed radius + margin); a gap narrower than that is only tried in `narrow` mode (half-width), which the
// report keeps separate because at 0.05 m map resolution the verdict there is below the map's own quantisation.
static Res run(const Grid& g, double sx, double sy, double sth, double gx, double gy, double gth) {
  Res r = run1(g, 0.372 + kMargin, sx, sy, sth, gx, gy, gth);
  if (r.reason == "no_geometric_path") { Res n = run1(g, kHalfW, sx, sy, sth, gx, gy, gth); if (n.result != "REJECT") { n.reason = n.result == "REACHED" ? "narrow_mode_passage" : n.reason + "+narrow_mode"; return n; } }
  return r;
}
int main(int argc, char** argv) {
  if (argc < 4) { fprintf(stderr, "usage: l1_runner <map_dir> <scenarios.tsv> <out.tsv> [threads]\n"); return 2; }
  Grid g; if (!load(argv[1], g)) { fprintf(stderr, "map load failed\n"); return 2; }
  struct S { std::string id, cat, exp; double v[6]; }; std::vector<S> sc; std::ifstream in(argv[2]); std::string ln;
  while (std::getline(in, ln)) { std::stringstream ss(ln); S s; std::getline(ss, s.id, '\t'); std::getline(ss, s.cat, '\t'); std::string f; for (int i = 0; i < 6; ++i) { std::getline(ss, f, '\t'); s.v[i] = atof(f.c_str()); } std::getline(ss, s.exp, '\t'); sc.push_back(s); }
  if (argc > 5) kVmax = atof(argv[5]);
  if (argc > 6) kMargin = atof(argv[6]);
  int nt = argc > 4 ? atoi(argv[4]) : 1; std::vector<Res> out(sc.size()); std::atomic<size_t> next{0};
  auto t0 = std::chrono::steady_clock::now();
  std::vector<std::thread> th; for (int k = 0; k < nt; ++k) th.emplace_back([&] { for (size_t i; (i = next++) < sc.size();) out[i] = run(g, sc[i].v[0], sc[i].v[1], sc[i].v[2], sc[i].v[3], sc[i].v[4], sc[i].v[5]); });
  for (auto& t : th) t.join();
  double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  FILE* o = fopen(argv[3], "w"); int ok = 0, bad = 0;
  for (size_t i = 0; i < sc.size(); ++i) {
    const Res& r = out[i]; bool expect_reach = sc[i].exp == "reach"; bool pass = expect_reach ? r.result == "REACHED" : r.result == "REJECT"; (pass ? ok : bad)++;
    fprintf(o, "%s\t%s\t%s\t%s\t%s\t%s\t%.2f\t%.2f\t%.3f\n", sc[i].id.c_str(), sc[i].cat.c_str(), sc[i].exp.c_str(), r.result.c_str(), pass ? "PASS" : "FAIL", r.reason.c_str(), r.t, r.len, r.minc > 1e8 ? -1 : r.minc);
  }
  fclose(o); printf("L1 MODEL: %zu scenarios, %d as expected, %d not, %.2fs wall, %.1f scen/s, %d threads\n", sc.size(), ok, bad, wall, sc.size() / wall, nt);
  return 0;
}
