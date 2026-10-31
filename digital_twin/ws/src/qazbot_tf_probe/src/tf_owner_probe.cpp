// SIM ONLY. Native Humble subscription metadata, no inference from topic/node listings.
#include "ownership.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std::chrono_literals;
static std::string hex(const uint8_t *p, std::size_t n) {
  std::ostringstream out;
  for (std::size_t i = 0; i < n; ++i) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(p[i]);
  return out.str();
}
int main(int argc, char **argv) {
  for (const auto &[key, value] : std::map<std::string, std::string>{{"QAZBOT_SIM", "1"}, {"TWIN_SANDBOX", "1"}, {"ROS_DOMAIN_ID", "87"}}) {
    const char *got = std::getenv(key.c_str());
    if (!got || value != got) { std::cerr << "refused: requires isolated SIM environment\n"; return 2; }
  }
  rclcpp::init(argc, argv);
  const std::string fixture = argc > 1 ? argv[1] : "";
  if (!fixture.empty() && fixture != "single" && fixture != "duplicate" && fixture != "same-name" && fixture != "missing") return 2;
  const bool test = !fixture.empty();
  auto node = std::make_shared<rclcpp::Node>("tf_owner_probe");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  tf_probe::Samples samples;
  std::map<std::string, std::set<std::string>> nodes;
  const char *owner_env = std::getenv("TWIN_ODOM_OWNER");
  const std::string odom_owner = owner_env ? owner_env : "/kinematic_plant";
  if (odom_owner != "/kinematic_plant" && odom_owner != "/hal_plant") return 2;
  const std::map<std::string, std::string> expected = {
    {"map->odom", test ? "/probe_amcl" : "/qazbot_d1_map/amcl"},
    {"odom->base_footprint", test ? "/probe_odom" : odom_owner}};
  const std::vector<std::string> topics = test ? std::vector<std::string>{"/qazbot_probe/test_tf"} :
    std::vector<std::string>{"/tf", "/qazbot_d1_map/tf", "/tf_static", "/qazbot_d1_map/tf_static"};
  std::vector<rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> subs;
  for (const auto &topic : topics) {
    auto qos = rclcpp::QoS(100).best_effort();
    if (topic.find("tf_static") != std::string::npos) qos.transient_local();
    subs.push_back(node->create_subscription<tf2_msgs::msg::TFMessage>(topic, qos,
      [&](const tf2_msgs::msg::TFMessage::ConstSharedPtr msg, const rclcpp::MessageInfo &info) {
        const auto &gid = info.get_rmw_message_info().publisher_gid;
        std::set<std::string> seen;
        for (const auto &tf : msg->transforms) {
          const auto edge = tf.header.frame_id + "->" + tf.child_frame_id;
          if (expected.count(edge)) seen.insert(edge);
        }
        for (const auto &edge : seen) ++samples[edge][hex(gid.data, RMW_GID_STORAGE_SIZE)];
      }));
  }
  // Endpoint names are joined to GIDs repeatedly so a publisher disappearing later cannot erase observed conflicts.
  auto discover = node->create_wall_timer(100ms, [&]() {
    for (const auto &topic : topics) for (const auto &info : node->get_publishers_info_by_topic(topic)) {
      const auto &gid = info.endpoint_gid();
      auto ns = info.node_namespace();
      if (ns == "/") ns.clear();
      nodes[hex(gid.data(), gid.size())].insert(ns + "/" + info.node_name());
    }
  });
  std::vector<rclcpp::Node::SharedPtr> fixtures;
  std::vector<rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr> pubs;
  std::vector<rclcpp::TimerBase::SharedPtr> timers;
  if (test && fixture != "missing") {
    const int count = (fixture == "single" ? 2 : 3);
    for (int i = 0; i < count; ++i) {
      fixtures.push_back(std::make_shared<rclcpp::Node>(i == 1 ? "probe_odom" : (i == 2 && fixture != "same-name" ? "intruder" : "probe_amcl")));
      executor.add_node(fixtures.back());
      auto pub = fixtures.back()->create_publisher<tf2_msgs::msg::TFMessage>(topics[0], 10);
      pubs.push_back(pub);
      timers.push_back(fixtures.back()->create_wall_timer(100ms, [pub, i]() {
        tf2_msgs::msg::TFMessage msg;
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = i == 1 ? "odom" : "map";
        tf.child_frame_id = i == 1 ? "base_footprint" : "odom";
        tf.transform.rotation.w = 1;
        msg.transforms.push_back(tf); pub->publish(msg);
      }));
    }
  }
  if (!test) {
    const char *start = std::getenv("TWIN_START");
    std::string input = start ? start : "";
    for (char &c : input) if (c == ',') c = ' ';
    double x, y, yaw; std::istringstream stream(input);
    if (!(stream >> x >> y >> yaw) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      std::cerr << "TWIN_START must contain three finite simulation coordinates\n"; return 2;
    }
    auto pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/qazbot_d1_map/initialpose", 10);
    timers.push_back(node->create_wall_timer(500ms, [&, pub, x, y, yaw]() {
      if (samples.count("map->odom")) return;
      geometry_msgs::msg::PoseWithCovarianceStamped pose;
      pose.header.frame_id = "map"; pose.header.stamp = node->now();
      pose.pose.pose.position.x = x; pose.pose.pose.position.y = y;
      pose.pose.pose.orientation.z = std::sin(yaw / 2); pose.pose.pose.orientation.w = std::cos(yaw / 2);
      pose.pose.covariance[0] = pose.pose.covariance[7] = pose.pose.covariance[35] = 0.05;
      pub->publish(pose);
    }));
  }
  const auto end = std::chrono::steady_clock::now() + (test ? 3s : 28s);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end) executor.spin_once(100ms);
  bool ok = tf_probe::valid(samples, nodes, expected);
  std::cout << "TFJSON{\"pass\":" << (ok ? "true" : "false") << ",\"owners\":{";
  bool first_edge = true;
  for (const auto &[edge, name] : expected) {
    (void)name;
    if (!first_edge) std::cout << ',';
    first_edge = false; std::cout << std::quoted(edge) << ":[";
    bool first_gid = true;
    for (const auto &[gid, count] : samples[edge]) {
      if (!first_gid) std::cout << ',';
      first_gid = false;
      std::cout << "{\"gid\":" << std::quoted(gid) << ",\"samples\":" << count << ",\"nodes\":[";
      bool first_node = true;
      for (const auto &name2 : nodes[gid]) {
        if (!first_node) std::cout << ',';
        first_node = false; std::cout << std::quoted(name2);
      }
      std::cout << "]}";
    }
    std::cout << ']';
  }
  std::cout << "}}\n";
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
