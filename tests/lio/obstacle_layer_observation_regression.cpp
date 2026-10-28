// Real installed Nav2 ObstacleLayer raytracing, without a ROS participant.
// Characterizes observation endpoints; does not validate cloud-to-scan geometry.
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class ObservationTestLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  using nav2_costmap_2d::ObstacleLayer::raytraceFreespace;
  ObservationTestLayer()
  {
    rolling_window_ = false;
    enabled_ = true;
    footprint_clearing_enabled_ = false;
    min_obstacle_height_ = 0.0;
    max_obstacle_height_ = 2.0;
  }
};

int main(int argc, char ** argv)
{
  using namespace nav2_costmap_2d;
  int checks = 0;
  auto check = [&checks](bool ok, const char * label) {
      if (!ok) {throw std::runtime_error(label);}
      ++checks;
      std::cout << "PASS " << label << '\n';
    };
  std::vector<double> endpoints{0.0, 1.0, 8.0};
  for (int i = 1; i < argc; ++i) {endpoints.push_back(std::stod(argv[i]));}
  for (const double endpoint : endpoints) {
    ObservationTestLayer layer;
    layer.resizeMap(200, 200, 0.05, -5.0, -5.0);
    auto cell = [&layer](double x, double y) {
        unsigned int mx, my;
        if (!layer.worldToMap(x, y, mx, my)) {throw std::runtime_error("bounds");}
        return std::make_pair(mx, my);
      };
    const auto near = cell(0.5, 0.0), far = cell(3.0, 0.0);
    const auto unknown = cell(3.0, 0.5), beyond = cell(4.0, 0.0);
    layer.setCost(near.first, near.second, LETHAL_OBSTACLE);
    layer.setCost(far.first, far.second, LETHAL_OBSTACLE);
    layer.setCost(unknown.first, unknown.second, NO_INFORMATION);
    layer.setCost(beyond.first, beyond.second, LETHAL_OBSTACLE);
    Observation obs;
    obs.origin_.x = obs.origin_.y = obs.origin_.z = 0.0;
    obs.raytrace_max_range_ = 3.5;
    obs.raytrace_min_range_ = 0.0;
    sensor_msgs::PointCloud2Modifier mod(*obs.cloud_);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(endpoint == 0.0 ? 0 : 1);
    if (endpoint != 0.0) {
      sensor_msgs::PointCloud2Iterator<float> x(*obs.cloud_, "x");
      sensor_msgs::PointCloud2Iterator<float> y(*obs.cloud_, "y");
      sensor_msgs::PointCloud2Iterator<float> z(*obs.cloud_, "z");
      *x = endpoint; *y = 0.0; *z = 0.0;
    }
    double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
    layer.raytraceFreespace(obs, &min_x, &min_y, &max_x, &max_y);
    check(layer.getCost(far.first, far.second) ==
      (endpoint >= 3.0 ? FREE_SPACE : LETHAL_OBSTACLE),
      endpoint == 8.0 ? "range-max endpoint ERASES distant lethal (unsafe coverage reproduced)" :
      endpoint == 0.0 ? "missing sensor preserves distant lethal" : "bounded endpoint preserves distant lethal");
    check(layer.getCost(near.first, near.second) ==
      (endpoint == 0.0 ? LETHAL_OBSTACLE : FREE_SPACE), "clearing bounded by actual endpoint");
    check(layer.getCost(unknown.first, unknown.second) == NO_INFORMATION,
      "unobserved neighbouring ray stays unknown");
    check(layer.getCost(beyond.first, beyond.second) == LETHAL_OBSTACLE,
      "cell beyond raytrace_max_range remains lethal");
  }
  // Exercise the actual layer update ordering: clearing first, then marking.
  // A current real obstacle must remain lethal even with a long clear ray.
  {
    ObservationTestLayer layer;
    layer.resizeMap(200, 200, 0.05, -5.0, -5.0);
    Observation clear, mark;
    for (auto * obs : {&clear, &mark}) {
      sensor_msgs::PointCloud2Modifier mod(*obs->cloud_);
      mod.setPointCloud2FieldsByString(1, "xyz");
      mod.resize(1);
      sensor_msgs::PointCloud2Iterator<float> x(*obs->cloud_, "x");
      sensor_msgs::PointCloud2Iterator<float> y(*obs->cloud_, "y");
      sensor_msgs::PointCloud2Iterator<float> z(*obs->cloud_, "z");
      *x = obs == &clear ? 8.0 : 0.825;
      *y = *z = 0.0;
      obs->origin_.x = obs->origin_.y = obs->origin_.z = 0;
      obs->raytrace_max_range_ = 3.5;
      obs->obstacle_max_range_ = 3.0;
    }
    layer.addStaticObservation(clear, false, true);
    layer.addStaticObservation(mark, true, false);
    double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
    layer.updateBounds(0, 0, 0, &x0, &y0, &x1, &y1);
    unsigned int mx, my;
    layer.worldToMap(0.825, 0.0, mx, my);
    check(layer.getCost(mx, my) == LETHAL_OBSTACLE,
      "actual updateBounds preserves current real obstacle after clearing");
  }
  std::cout << checks << " checks passed; old coverage safety remains FAIL\n";
}
