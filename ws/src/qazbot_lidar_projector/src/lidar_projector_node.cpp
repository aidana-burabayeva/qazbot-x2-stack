// Copyright 2026 Qazbot
#include "qazbot_lidar_projector/lidar_projector_node.hpp"

#include <cmath>
#include <cstring>
#include <utility>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace qazbot_lidar_projector
{

namespace
{
using sensor_msgs::msg::PointField;

CoreParams declareCoreParams(rclcpp::Node & node)
{
  CoreParams p;
  p.range_max = node.declare_parameter("range_max", p.range_max);
  p.range_min = node.declare_parameter("range_min", p.range_min);
  p.z_min = node.declare_parameter("z_min", p.z_min);
  p.z_max = node.declare_parameter("z_max", p.z_max);
  p.floor_margin_per_m = node.declare_parameter("floor_margin_per_m", p.floor_margin_per_m);
  p.n_bins = static_cast<int>(node.declare_parameter("n_bins", static_cast<int64_t>(p.n_bins)));
  p.level_s = node.declare_parameter("level_s", p.level_s);
  p.twist_baseline_s = node.declare_parameter("twist_baseline_s", p.twist_baseline_s);
  const auto fwd = node.declare_parameter(
    "lidar_fwd_in_imu",
    std::vector<double>{p.lidar_fwd_in_imu.x, p.lidar_fwd_in_imu.y, p.lidar_fwd_in_imu.z});
  if (fwd.size() == 3) {
    p.lidar_fwd_in_imu = {fwd[0], fwd[1], fwd[2]};
  } else {
    RCLCPP_WARN(node.get_logger(), "lidar_fwd_in_imu needs 3 values, keeping the default");
  }
  if (p.n_bins < 1) {
    RCLCPP_WARN(node.get_logger(), "n_bins %d < 1, using 1440", p.n_bins);
    p.n_bins = 1440;
  }
  return p;
}

bool findXyz(const sensor_msgs::msg::PointCloud2 & m, std::size_t & ox, std::size_t & oy, std::size_t & oz)
{
  int found = 0;
  for (const auto & f : m.fields) {
    if (f.datatype != PointField::FLOAT32 || f.count != 1) {continue;}
    if (f.name == "x") {ox = f.offset; found |= 1;}
    if (f.name == "y") {oy = f.offset; found |= 2;}
    if (f.name == "z") {oz = f.offset; found |= 4;}
  }
  return found == 7;
}
}  // namespace

LidarProjectorNode::LidarProjectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("lidar_projector", options),
  core_(declareCoreParams(*this)),
  publish_tf_(declare_parameter("publish_tf", true)),
  scan_every_(static_cast<int>(declare_parameter("scan_every", static_cast<int64_t>(1)))),
  leveller_(core_.level_s),
  tracker_(core_.twist_baseline_s),
  processor_(core_)
{
  const std::string imu_topic =
    declare_parameter("imu_topic", std::string("/aima/hal/sensor/lidar_chest_front/imu"));
  if (scan_every_ < 1) {
    RCLCPP_WARN(get_logger(), "scan_every %d < 1, using 1", scan_every_);
    scan_every_ = 1;
  }
  // publish_tf false: no TF publisher at all (single odom->base_footprint owner, checked on the graph).
  if (publish_tf_) {
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::QoS(400).best_effort(),
    [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {onImu(m);});
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "/Odometry", 100, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {onOdom(m);});
  sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/cloud_registered", 20, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {onCloud(m);});
  pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 20);
  // Follow the resolved/remapped scan topic.
  const std::string scan_topic = pub_scan_->get_topic_name();
  pub_clearing_ = create_publisher<sensor_msgs::msg::PointCloud2>(scan_topic + "_clearing", 20);
  pub_obstacles_ = create_publisher<sensor_msgs::msg::PointCloud2>(scan_topic + "_obstacles", 20);
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odom_level", 50);
}

void LidarProjectorNode::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr & m)
{
  if (leveller_.ready() || !leveller_.hasOrientation()) {
    return;
  }
  const double t = m->header.stamp.sec + m->header.stamp.nanosec * 1e-9;
  const auto & a = m->linear_acceleration;
  if (leveller_.addAccel(t, {a.x, a.y, a.z})) {
    const Vec3 & up = leveller_.up();
    RCLCPP_INFO(get_logger(), "levelled: up in FAST-LIO world [%.4f %.4f %.4f]", up.x, up.y, up.z);
    // Done with the IMU: stop receiving 200 msg/s that would all be ignored.
    sub_imu_.reset();
  }
}

void LidarProjectorNode::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr & m)
{
  const auto & q = m->pose.pose.orientation;
  const Mat3 r_wb = quatToRot(q.x, q.y, q.z, q.w);
  leveller_.setOrientation(r_wb);
  if (!leveller_.ready()) {
    return;
  }
  const auto & pp = m->pose.pose.position;
  const Pose pose = levelPose(leveller_.rotation(), r_wb, {pp.x, pp.y, pp.z}, core_.lidar_fwd_in_imu);
  const std::int64_t key = static_cast<std::int64_t>(m->header.stamp.sec) * 1000000000LL +
    m->header.stamp.nanosec;
  const double stamp = m->header.stamp.sec + m->header.stamp.nanosec * 1e-9;
  const auto upd = tracker_.push(key, stamp, pose);
  if (upd.clock_stepped_back) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "odometry stamp went back %.3f s: clearing pose cache and twist baseline", upd.step_back_s);
  }
  const double qz = std::sin(pose.yaw / 2), qw = std::cos(pose.yaw / 2);
  if (tf_) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = m->header.stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = frame_base_;
    t.transform.translation.x = pose.p_lv.x;
    t.transform.translation.y = pose.p_lv.y;
    t.transform.rotation.z = qz;
    t.transform.rotation.w = qw;
    tf_->sendTransform(t);
  }
  if (!upd.has_twist) {
    return;
  }
  auto o = std::make_unique<nav_msgs::msg::Odometry>();
  o->header.stamp = m->header.stamp;
  o->header.frame_id = "odom";
  o->child_frame_id = frame_base_;
  o->pose.pose.position.x = pose.p_lv.x;
  o->pose.pose.position.y = pose.p_lv.y;
  o->pose.pose.position.z = pose.p_lv.z;
  o->pose.pose.orientation.z = qz;
  o->pose.pose.orientation.w = qw;
  o->twist.twist.linear.x = upd.twist.vx;
  o->twist.twist.linear.y = upd.twist.vy;
  o->twist.twist.angular.z = upd.twist.wz;
  Mat6 cov;
  std::copy(m->pose.covariance.begin(), m->pose.covariance.end(), cov.begin());
  const Mat6 rc = rotateCovariance(leveller_.rotation(), cov);
  std::copy(rc.begin(), rc.end(), o->pose.covariance.begin());
  pub_odom_->publish(std::move(o));
}

void LidarProjectorNode::onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & m)
{
  ++n_cloud_;
  if (n_cloud_ % static_cast<std::uint64_t>(scan_every_)) {
    return;
  }
  const std::int64_t key = static_cast<std::int64_t>(m->header.stamp.sec) * 1000000000LL +
    m->header.stamp.nanosec;
  const Pose * pose = tracker_.find(key);
  if (pose == nullptr || !leveller_.ready()) {
    return;
  }
  XyzView view;
  if (m->is_bigendian || !findXyz(*m, view.off_x, view.off_y, view.off_z)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "cloud without little-endian float32 x/y/z fields dropped");
    return;
  }
  view.data = m->data.data();
  view.count = static_cast<std::size_t>(m->width) * m->height;
  view.step = m->point_step;
  view.skip_nans = !m->is_dense;
  if (view.step < 12 || m->data.size() < view.count * view.step) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "malformed cloud (size/point_step) dropped");
    return;
  }
  if (!processor_.process(view, leveller_.rotation(), *pose, result_)) {
    return;
  }
  auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan->header.stamp = m->header.stamp;
  scan->header.frame_id = frame_base_;
  scan->angle_min = -M_PI;
  scan->angle_increment = 2 * M_PI / core_.n_bins;
  scan->angle_max = scan->angle_min + scan->angle_increment * (core_.n_bins - 1);
  scan->range_min = static_cast<float>(core_.range_min);
  scan->range_max = static_cast<float>(core_.range_max);
  scan->ranges = result_.ranges;
  const auto header = scan->header;
  pub_scan_->publish(std::move(scan));
  pub_obstacles_->publish(makeXyzCloud(header, result_.obstacles_xyz));
  pub_clearing_->publish(makeXyzCloud(header, result_.clearing_xyz));
}

sensor_msgs::msg::PointCloud2::UniquePtr LidarProjectorNode::makeXyzCloud(
  const std_msgs::msg::Header & header, const std::vector<float> & xyz) const
{
  auto c = std::make_unique<sensor_msgs::msg::PointCloud2>();
  c->header = header;
  const auto n = static_cast<std::uint32_t>(xyz.size() / 3);
  c->height = 1;
  c->width = n;
  const char * names[3] = {"x", "y", "z"};
  for (std::uint32_t i = 0; i < 3; ++i) {
    PointField f;
    f.name = names[i];
    f.offset = 4 * i;
    f.datatype = PointField::FLOAT32;
    f.count = 1;
    c->fields.push_back(f);
  }
  c->is_bigendian = false;
  c->point_step = 12;
  c->row_step = 12 * n;
  c->is_dense = false;
  c->data.resize(xyz.size() * sizeof(float));
  if (!xyz.empty()) {
    std::memcpy(c->data.data(), xyz.data(), c->data.size());
  }
  return c;
}

}  // namespace qazbot_lidar_projector

RCLCPP_COMPONENTS_REGISTER_NODE(qazbot_lidar_projector::LidarProjectorNode)
