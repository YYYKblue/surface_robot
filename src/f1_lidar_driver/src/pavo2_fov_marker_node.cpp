#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double degrees_to_radians(double degrees)
{
  return degrees * kPi / 180.0;
}

geometry_msgs::msg::Point polar_point(double range, double angle, double z)
{
  geometry_msgs::msg::Point point;
  point.x = range * std::cos(angle);
  point.y = range * std::sin(angle);
  point.z = z;
  return point;
}

geometry_msgs::msg::Point origin_point(double z)
{
  geometry_msgs::msg::Point point;
  point.z = z;
  return point;
}
}  // namespace

class Pavo2FovMarkerNode : public rclcpp::Node
{
public:
  Pavo2FovMarkerNode()
  : Node("pavo2_fov_marker_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "pavo2s_frame");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/pavo2_scan_visualization");
    range_m_ = std::max(0.1, declare_parameter<double>("range_m", 100.0));
    angle_min_deg_ = declare_parameter<double>("angle_min_deg", -140.0);
    angle_max_deg_ = declare_parameter<double>("angle_max_deg", 140.0);
    arc_step_deg_ = std::max(0.1, declare_parameter<double>("arc_step_deg", 1.0));

    if (angle_min_deg_ > angle_max_deg_) {
      std::swap(angle_min_deg_, angle_max_deg_);
    }

    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(1).transient_local().reliable());
    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&Pavo2FovMarkerNode::publish_markers, this));

    publish_markers();
    RCLCPP_INFO(
      get_logger(), "Publishing %.1f m FOV markers from %.1f to %.1f degrees on %s.",
      range_m_, angle_min_deg_, angle_max_deg_, marker_topic_.c_str());
  }

private:
  visualization_msgs::msg::Marker make_marker(int id, int type) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "pavo2_fov";
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.frame_locked = true;
    return marker;
  }

  visualization_msgs::msg::Marker make_sector_fill() const
  {
    auto marker = make_marker(0, visualization_msgs::msg::Marker::TRIANGLE_LIST);
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.08F;
    marker.color.g = 0.55F;
    marker.color.b = 0.95F;
    marker.color.a = 0.08F;

    for (double angle = angle_min_deg_; angle < angle_max_deg_; angle += arc_step_deg_) {
      const double next_angle = std::min(angle + arc_step_deg_, angle_max_deg_);
      marker.points.push_back(origin_point(-0.04));
      marker.points.push_back(polar_point(range_m_, degrees_to_radians(angle), -0.04));
      marker.points.push_back(polar_point(range_m_, degrees_to_radians(next_angle), -0.04));
    }
    return marker;
  }

  visualization_msgs::msg::Marker make_sector_outline() const
  {
    auto marker = make_marker(1, visualization_msgs::msg::Marker::LINE_STRIP);
    marker.scale.x = 0.02;
    marker.color.r = 0.10F;
    marker.color.g = 0.80F;
    marker.color.b = 1.00F;
    marker.color.a = 0.95F;

    marker.points.push_back(origin_point(0.04));
    marker.points.push_back(
      polar_point(range_m_, degrees_to_radians(angle_min_deg_), 0.04));
    for (double angle = angle_min_deg_; angle < angle_max_deg_; angle += arc_step_deg_) {
      const double next_angle = std::min(angle + arc_step_deg_, angle_max_deg_);
      marker.points.push_back(polar_point(range_m_, degrees_to_radians(next_angle), 0.04));
    }
    marker.points.push_back(origin_point(0.04));
    return marker;
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(make_sector_fill());
    markers.markers.push_back(make_sector_outline());
    publisher_->publish(markers);
  }

  std::string frame_id_;
  std::string marker_topic_;
  double range_m_;
  double angle_min_deg_;
  double angle_max_deg_;
  double arc_step_deg_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pavo2FovMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
