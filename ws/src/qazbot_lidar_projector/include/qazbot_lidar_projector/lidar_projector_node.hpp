// Copyright 2026 Qazbot
// Thin ROS 2 node around the pure lidar_projection library: subscriptions, publishers, parameters, TF.
#ifndef QAZBOT_LIDAR_PROJECTOR__LIDAR_PROJECTOR_NODE_HPP_
#define QAZBOT_LIDAR_PROJECTOR__LIDAR_PROJECTOR_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "qazbot_lidar_projector/lidar_projection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace qazbot_lidar_projector
{

class LidarProjectorNode : public rclcpp::Node
{
public:
  explicit LidarProjectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  sensor_msgs::msg::PointCloud2::UniquePtr makeXyzCloud(
    const std_msgs::msg::Header & header, const std::vector<float> & xyz) const;

  CoreParams core_;
  bool publish_tf_;
  int scan_every_;
  std::uint64_t n_cloud_{0};
  std::string frame_base_{"base_footprint"};

  GravityLeveller leveller_;
  PoseTracker tracker_;
  CloudProcessor processor_;
  CloudResult result_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clearing_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
};

}  // namespace qazbot_lidar_projector

#endif  // QAZBOT_LIDAR_PROJECTOR__LIDAR_PROJECTOR_NODE_HPP_
