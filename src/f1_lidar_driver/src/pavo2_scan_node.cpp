#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "pavo2_driver.h"
#include "pavo2_utils.h"

namespace
{
constexpr float kDistanceUnit = 0.002F;
constexpr double kMinSupportedRange = 0.10;
constexpr double kMaxSupportedRange = 100.0;
constexpr int kDefaultTimeoutMs = 180;
constexpr double kPi = 3.14159265358979323846;

double degrees_to_radians(double degrees)
{
  return degrees * kPi / 180.0;
}

void publish_scan(
  const rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr & publisher,
  const std::vector<pavo2::pavo_response_scan_t> & points,
  const rclcpp::Time & stamp, double scan_time, const std::string & frame_id,
  bool inverted, double range_min, double range_max)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = frame_id;
  scan.ranges.resize(points.size());
  scan.intensities.resize(points.size());

  for (std::size_t i = 0; i < points.size(); ++i) {
    float range = points[i].distance * kDistanceUnit;
    float intensity = points[i].intensity;
    if (range < range_min || range > range_max) {
      range = 0.0F;
      intensity = 0.0F;
    }
    const auto output_index = inverted ? points.size() - 1U - i : i;
    scan.ranges[output_index] = range;
    scan.intensities[output_index] = intensity;
  }

  scan.angle_min = degrees_to_radians(points.front().angle / 100.0) - kPi;
  scan.angle_max = degrees_to_radians(points.back().angle / 100.0) - kPi;
  scan.angle_increment = (scan.angle_max - scan.angle_min) / (points.size() - 1U);
  scan.scan_time = scan_time;
  scan.time_increment = scan_time * scan.angle_increment / (2.0 * kPi);
  scan.range_min = range_min;
  scan.range_max = range_max;
  publisher->publish(scan);
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared("pavo2_scan_node");

  const auto frame_id = node->declare_parameter<std::string>("frame_id", "pavo2_frame");
  const auto scan_topic = node->declare_parameter<std::string>("topic", "pavo2_scan");
  auto range_min = node->declare_parameter<double>("range_min", kMinSupportedRange);
  auto range_max = node->declare_parameter<double>("range_max", kMaxSupportedRange);
  const auto inverted = node->declare_parameter<bool>("inverted", false);
  int timeout_ms = static_cast<int>(
    node->declare_parameter<int64_t>("timeout", kDefaultTimeoutMs));

  if (range_min < kMinSupportedRange || range_min >= range_max) {
    RCLCPP_WARN(node->get_logger(), "Invalid range_min [%f]; using %f.", range_min, kMinSupportedRange);
    range_min = kMinSupportedRange;
  }
  if (range_max > kMaxSupportedRange || range_max <= range_min) {
    RCLCPP_WARN(node->get_logger(), "Invalid range_max [%f]; using %f.", range_max, kMaxSupportedRange);
    range_max = kMaxSupportedRange;
  }
  if (timeout_ms < 0 || timeout_ms > 65535) {
    RCLCPP_WARN(node->get_logger(), "Invalid timeout [%d]; using %d ms.", timeout_ms, kDefaultTimeoutMs);
    timeout_ms = kDefaultTimeoutMs;
  }

  auto driver = init_lidar(node, static_cast<uint16_t>(timeout_ms));
  if (!driver) {
    rclcpp::shutdown();
    return 1;
  }

  uint8_t motor_speed = 0;
  if (!driver->get_motor_speed(motor_speed) || motor_speed == 0) {
    RCLCPP_FATAL(node->get_logger(), "Could not read a valid motor speed from Pavo2.");
    rclcpp::shutdown();
    return 1;
  }
  if (!driver->enable_data(true)) {
    RCLCPP_FATAL(node->get_logger(), "Pavo2 rejected the command to enable scan data.");
    rclcpp::shutdown();
    return 1;
  }

  const auto publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::QoS(10).reliable());
  const double scan_time = 1.0 / static_cast<double>(motor_speed);
  const auto lidar_ip = node->get_parameter("lidar_ip").as_string();
  const auto lidar_port = node->get_parameter("lidar_port").as_int();

  RCLCPP_INFO(
    node->get_logger(), "Publishing /%s in frame %s at approximately %u Hz.",
    scan_topic.c_str(), frame_id.c_str(), static_cast<unsigned int>(motor_speed));

  unsigned int consecutive_errors = 0;
  while (rclcpp::ok()) {
    std::vector<pavo2::pavo_response_scan_t> points;
    const auto stamp = node->now();
    if (!driver->get_scanned_data(points, static_cast<uint16_t>(timeout_ms))) {
      ++consecutive_errors;
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "No scan frame received from Pavo2 (%u consecutive timeouts).", consecutive_errors);

      if (consecutive_errors > 6U) {
        driver->pavo2_close();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (driver->pavo2_open(
            lidar_ip, static_cast<uint16_t>(lidar_port), static_cast<uint16_t>(timeout_ms)) &&
          driver->enable_data(true))
        {
          RCLCPP_INFO(node->get_logger(), "Reconnected to Pavo2.");
          consecutive_errors = 0;
        }
      }
      if (rclcpp::ok()) {
        rclcpp::spin_some(node);
      }
      continue;
    }

    consecutive_errors = 0;
    if (points.size() >= 2U) {
      publish_scan(
        publisher, points, stamp, scan_time, frame_id,
        inverted, range_min, range_max);
    }
    if (rclcpp::ok()) {
      rclcpp::spin_some(node);
    }
  }

  driver->enable_data(false);
  driver->pavo2_close();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
