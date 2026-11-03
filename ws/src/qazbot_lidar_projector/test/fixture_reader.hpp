// Copyright 2026 Qazbot
// Reader of the binary fixture written by test/tools/fixture.py (layout documented there).
#ifndef FIXTURE_READER_HPP_
#define FIXTURE_READER_HPP_

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

namespace fixture
{
struct Event
{
  int kind;   // 0 imu, 1 odom, 2 cloud
  std::int32_t sec;
  std::uint32_t nanosec;
  double v[43];             // imu: 3 (accel); odom: 3 pos + 4 quat + 36 cov
  bool dense{true};
  std::vector<float> xyz;   // cloud: 3 floats per point
};

inline std::vector<Event> read(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {throw std::runtime_error("cannot open " + path);}
  std::vector<char> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  if (buf.size() < 12 || std::memcmp(buf.data(), "LBFX1\0\0\0", 8) != 0) {throw std::runtime_error("bad fixture");}
  std::size_t pos = 8;
  auto get = [&](void * dst, std::size_t n) {
      if (pos + n > buf.size()) {throw std::runtime_error("truncated fixture");}
      std::memcpy(dst, buf.data() + pos, n);
      pos += n;
    };
  std::uint32_t n;
  get(&n, 4);
  std::vector<Event> out(n);
  for (auto & e : out) {
    std::uint8_t k;
    get(&k, 1); get(&e.sec, 4); get(&e.nanosec, 4);
    e.kind = k;
    if (k == 0) {
      get(e.v, 24);
    } else if (k == 1) {
      get(e.v, 43 * 8);
    } else {
      std::uint8_t d; std::uint32_t cnt;
      get(&d, 1); get(&cnt, 4);
      e.dense = d != 0;
      e.xyz.resize(3u * cnt);
      get(e.xyz.data(), 12u * cnt);
    }
  }
  return out;
}
}  // namespace fixture

#endif  // FIXTURE_READER_HPP_
