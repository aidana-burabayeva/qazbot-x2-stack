#pragma once
#include <map>
#include <set>
#include <string>
#include <cstddef>

namespace tf_probe {
using Samples = std::map<std::string, std::map<std::string, std::size_t>>;
// Keep GIDs, not deduplicated node names: two endpoints with the same ROS name still compete.
inline bool valid(const Samples &samples, const std::map<std::string, std::set<std::string>> &nodes,
                  const std::map<std::string, std::string> &expected, std::size_t minimum = 3) {
  for (const auto &[edge, name] : expected) {
    auto e = samples.find(edge);
    if (e == samples.end() || e->second.size() != 1) return false;
    const auto &[gid, count] = *e->second.begin();
    auto n = nodes.find(gid);
    if (gid.empty() || gid.find_first_not_of('0') == std::string::npos || count < minimum ||
        n == nodes.end() || n->second != std::set<std::string>{name}) return false;
  }
  return true;
}
}  // namespace tf_probe
