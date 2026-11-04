// Copyright 2026 Qazbot
// Pure (ROS-free) core of lidar_projector (replaces ws/src/lio/level_bridge.py): gravity levelling from the LiDAR IMU, levelled planar pose, twist from a
// pose baseline, height-band classification, bearing bins, LaserScan ranges, obstacle points and clearing endpoints.
// Behaviour is a line-by-line port of candidates/d04/level_bridge.py (D-04 variant C); see README.md.
#ifndef QAZBOT_LIDAR_PROJECTOR__LIDAR_PROJECTION_HPP_
#define QAZBOT_LIDAR_PROJECTOR__LIDAR_PROJECTION_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <vector>

namespace qazbot_lidar_projector
{

struct Vec3
{
  double x{0.0}, y{0.0}, z{0.0};
};

/// Row-major 3x3 matrix.
using Mat3 = std::array<double, 9>;
/// Row-major 6x6 matrix (pose covariance: x y z roll pitch yaw).
using Mat6 = std::array<double, 36>;

Mat3 identity3();
Mat3 matMul(const Mat3 & a, const Mat3 & b);
Vec3 matVec(const Mat3 & a, const Vec3 & v);
/// Rotation matrix of the quaternion (x, y, z, w), NOT normalised (as the Python reference).
Mat3 quatToRot(double x, double y, double z, double w);
/// Rotation taking `up` (FAST-LIO world) onto +z; rows are the new x, y, z axes.
Mat3 levelRotation(const Vec3 & up);
/// B * cov * B^T with B = blockdiag(R, R).
Mat6 rotateCovariance(const Mat3 & r, const Mat6 & cov);

/// Parameters of the pure core. Defaults are the module constants of the Python reference.
struct CoreParams
{
  double range_min{0.45};              ///< [m]
  double range_max{25.0};              ///< [m] scan/obstacle/clearing range limit
  double z_min{-0.65};                 ///< [m] slice lower bound relative to the LiDAR IMU
  double z_max{0.80};                  ///< [m] slice upper bound relative to the LiDAR IMU
  double floor_margin_per_m{0.035};    ///< [m/m] lower slice bound rises with range (sway margin)
  int n_bins{1440};                    ///< bearing bins over [-pi, pi)
  double level_s{1.0};                 ///< [s] IMU averaging window for levelling
  double twist_baseline_s{0.3};        ///< [s] pose baseline for the twist
  /// LiDAR forward axis in the LiDAR IMU frame (column 0 of R_imu<-lidar).
  Vec3 lidar_fwd_in_imu{-0.003843965609489519, 0.0008810920694298785, -0.9999922237723444};
};

/// Averages gravity (IMU acceleration rotated into the FAST-LIO world) for level_s seconds, then fixes R_lv.
class GravityLeveller
{
public:
  explicit GravityLeveller(double level_s);
  /// Latest FAST-LIO world<-body rotation (from the odometry orientation).
  void setOrientation(const Mat3 & r_wb);
  bool hasOrientation() const {return has_orientation_;}
  bool ready() const {return ready_;}
  /// Feeds one IMU sample (stamp in seconds, body-frame acceleration). Ignored once ready or before the first
  /// orientation. Returns true on the call that completes levelling.
  bool addAccel(double stamp_s, const Vec3 & accel_body);
  const Mat3 & rotation() const {return r_lv_;}
  /// Unit gravity-up vector in the FAST-LIO world (valid once ready).
  const Vec3 & up() const {return up_;}

private:
  double level_s_;
  Mat3 r_wb_{};
  bool has_orientation_{false};
  bool ready_{false};
  bool has_t0_{false};
  double t0_{0.0};
  Vec3 sum_{};
  std::size_t count_{0};
  Mat3 r_lv_{};
  Vec3 up_{};
};

/// Levelled planar pose of one odometry message.
struct Pose
{
  double x{0.0}, y{0.0};   ///< levelled position [m]
  double yaw{0.0};         ///< heading of the LiDAR forward axis on the floor [rad]
  Vec3 p_lv{};             ///< full levelled position [m]
};

/// p_lv = R_lv p; yaw = atan2 of R_lv R_wb * fwd.
Pose levelPose(const Mat3 & r_lv, const Mat3 & r_wb, const Vec3 & p_world, const Vec3 & lidar_fwd_in_imu);

struct Twist
{
  double vx{0.0}, vy{0.0}, wz{0.0};   ///< in the child frame [m/s, m/s, rad/s]
};

/// Stamp-keyed pose cache (newest 50) plus the twist baseline track; handles a sensor clock stepping back.
class PoseTracker
{
public:
  struct Update
  {
    bool clock_stepped_back{false};
    double step_back_s{0.0};
    bool has_twist{false};
    Twist twist{};
  };
  explicit PoseTracker(double twist_baseline_s, std::size_t max_poses = 50);
  /// key_ns: stamp in integer nanoseconds (cache key); stamp_s: the same stamp in seconds (twist maths).
  Update push(std::int64_t key_ns, double stamp_s, const Pose & pose);
  const Pose * find(std::int64_t key_ns) const;
  std::size_t size() const {return poses_.size();}
  /// True once any odometry was pushed; newestKey() is then the stamp of the last pushed one.
  bool hasNewest() const {return has_last_;}
  std::int64_t newestKey() const {return last_key_;}

private:
  struct Sample
  {
    double t, x, y, yaw;
  };
  double baseline_s_;
  std::size_t max_poses_;
  bool has_last_{false};
  std::int64_t last_key_{0};
  std::map<std::int64_t, Pose> poses_;
  std::deque<Sample> track_;
};

/// Matches a cloud with the odometry of the SAME stamp when the two topics are delivered in either order.
/// FAST-LIO publishes /Odometry before /cloud_registered, but they are separate DDS readers, so a cloud can be taken
/// before its odometry. Instead of dropping that cloud, at most ONE cloud is held until the next odometry decides it:
/// equal stamp -> release (process now), newer stamp -> the held cloud's odometry was skipped -> drop.
/// Purely event-driven (no clock, no timeout); a newer deferred cloud replaces an older held one (counted as dropped).
class CloudPoseSync
{
public:
  enum class CloudAction {kProcess, kDefer, kDrop};
  enum class OdomAction {kNone, kRelease, kDropPending};
  struct Counters
  {
    std::uint64_t processed_direct{0};   ///< pose already known on cloud arrival
    std::uint64_t deferred{0};           ///< cloud held, waiting for its odometry
    std::uint64_t released{0};           ///< held cloud matched by a later odometry
    std::uint64_t dropped_no_pose{0};    ///< pose older than the newest odometry and absent: never arrives
    std::uint64_t dropped_pending{0};    ///< held cloud superseded by a newer odometry or a newer cloud
  };
  /// pose_known: tracker has the exact stamp. has_odom/newest_key: newest odometry stamp seen so far.
  CloudAction onCloud(std::int64_t key, bool pose_known, bool has_odom, std::int64_t newest_key);
  /// Call after every odometry was pushed into the tracker.
  OdomAction onOdom(std::int64_t key);
  bool hasPending() const {return has_pending_;}
  std::int64_t pendingKey() const {return pending_key_;}
  const Counters & counters() const {return counters_;}

private:
  bool has_pending_{false};
  std::int64_t pending_key_{0};
  Counters counters_{};
};

/// View on the x/y/z float32 fields of a packed point cloud (no ROS types).
struct XyzView
{
  const std::uint8_t * data{nullptr};
  std::size_t count{0};       ///< points (width * height)
  std::size_t step{0};        ///< bytes per point
  std::size_t off_x{0}, off_y{0}, off_z{0};
  bool skip_nans{false};      ///< drop points with a NaN coordinate (PointCloud2 is_dense == false)
};

/// Points in the body frame (levelled, origin at the LiDAR IMU, rotated by -yaw), struct-of-arrays.
struct BodyPoints
{
  std::vector<double> bx, by, r, a, z, flat;
  void clear();
  void reserve(std::size_t n);
  std::size_t size() const {return bx.size();}
  /// Adds one levelled point (q = R_lv P - p_lv); non-finite points are dropped. c = cos(-yaw), s = sin(-yaw).
  void push(double qx, double qy, double qz, double c, double s);
};

/// Bin of bearing `a` in [-pi, pi]: ((a + pi) / 2pi * n) truncated, modulo n.
int binIndex(double a, int n_bins);
/// Points of the slice: z_min + margin * flat < z < z_max.
bool inBand(double z, double flat, const CoreParams & p);

/// q = R_lv P - p_lv for every (non-skipped, finite) point of the view, rotated into the body frame by -yaw.
/// Returns the number of points read from the cloud after NaN skipping (0 means: publish nothing).
std::size_t prepareBodyPoints(const XyzView & cloud, const Mat3 & r_lv, const Pose & pose, BodyPoints & out);

/// LaserScan ranges (size n_bins, +inf where empty) and obstacle points (x, y, 0 per non-empty bin, bin order):
/// nearest in-slice return per bin within (range_min, range_max).
void scanFromBody(const BodyPoints & pts, const CoreParams & p, std::vector<float> & ranges,
  std::vector<float> & obstacles_xyz);

/// Conservative projected ray endpoints (x, y, 0 per bin, bin order): longest observed free ray per bin, capped
/// just short of the nearest in-slice hit; see the Python reference docstring.
void clearingFromBody(const BodyPoints & pts, const CoreParams & p, std::vector<float> & clearing_xyz);

/// Output buffers of one cloud, reused between calls.
struct CloudResult
{
  std::vector<float> ranges;
  std::vector<float> obstacles_xyz;
  std::vector<float> clearing_xyz;
};

/// Owns the per-cloud scratch buffers so that steady-state processing does not allocate.
class CloudProcessor
{
public:
  explicit CloudProcessor(const CoreParams & params);
  const CoreParams & params() const {return params_;}
  /// Returns false (outputs untouched) when the cloud has no usable points.
  bool process(const XyzView & cloud, const Mat3 & r_lv, const Pose & pose, CloudResult & out);

private:
  CoreParams params_;
  BodyPoints body_;
};

}  // namespace qazbot_lidar_projector

#endif  // QAZBOT_LIDAR_PROJECTOR__LIDAR_PROJECTION_HPP_
