#include "pavo2_utils.h"

#include <arpa/inet.h>

#include <chrono>
#include <string>
#include <thread>

std::unique_ptr<pavo2::pavo2_driver> init_lidar(
  const rclcpp::Node::SharedPtr & node, uint16_t timeout_ms)
{
  const auto local_ip = node->declare_parameter<std::string>("local_ip", "");
  const int local_port = static_cast<int>(node->declare_parameter<int64_t>("local_port", 0));
  const auto lidar_ip = node->declare_parameter<std::string>("lidar_ip", "10.10.10.121");
  const int lidar_port = static_cast<int>(node->declare_parameter<int64_t>("lidar_port", 2368));
  int protocol = static_cast<int>(node->declare_parameter<int64_t>("conn_protocol", 0));

  in_addr address{};
  if (inet_pton(AF_INET, lidar_ip.c_str(), &address) != 1) {
    RCLCPP_FATAL(node->get_logger(), "Invalid lidar_ip [%s].", lidar_ip.c_str());
    return nullptr;
  }
  if (!local_ip.empty() && inet_pton(AF_INET, local_ip.c_str(), &address) != 1) {
    RCLCPP_FATAL(node->get_logger(), "Invalid local_ip [%s].", local_ip.c_str());
    return nullptr;
  }
  if (lidar_port < 1 || lidar_port > 65535 || local_port < 0 || local_port > 65535) {
    RCLCPP_FATAL(
      node->get_logger(), "Invalid port: local_port [%d], lidar_port [%d].",
      local_port, lidar_port);
    return nullptr;
  }
  if (protocol < 0 || protocol > 2) {
    RCLCPP_WARN(node->get_logger(), "Invalid conn_protocol [%d]; using TCP/default [0].", protocol);
    protocol = 0;
  }

  auto driver = std::make_unique<pavo2::pavo2_driver>();
  driver->set_local_address(local_ip, static_cast<uint16_t>(local_port));
  driver->set_conn_protocol(static_cast<pavo2::PAVO2_PROTO_TYPE>(protocol));

  RCLCPP_INFO(
    node->get_logger(), "Connecting to Pavo2 at %s:%d using protocol %d.",
    lidar_ip.c_str(), lidar_port, protocol);
  if (!driver->pavo2_open(lidar_ip, static_cast<uint16_t>(lidar_port), timeout_ms)) {
    RCLCPP_FATAL(node->get_logger(), "Could not connect to Pavo2 at %s:%d.", lidar_ip.c_str(), lidar_port);
    return nullptr;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  RCLCPP_INFO(node->get_logger(), "Connected to Pavo2 successfully.");
  return driver;
}
