#ifndef PAVO2_ROS2_UTILS_H_
#define PAVO2_ROS2_UTILS_H_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "pavo2_driver.h"

std::unique_ptr<pavo2::pavo2_driver> init_lidar(
  const rclcpp::Node::SharedPtr & node, uint16_t timeout_ms);

#endif  // PAVO2_ROS2_UTILS_H_
