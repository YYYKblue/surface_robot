// Exercise the migrated production conversion without opening any sensor socket.
// Reuse the driver conversion with synthetic raw SDK samples.
#define main imported_pavo2_driver_main
#include "../src/pavo2_scan_node.cpp"
#undef main

#include <gtest/gtest.h>

class ScanConversion : public ::testing::TestWithParam<bool>
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_P(ScanConversion, PreservesCoordinatesTimingRangesIntensityAndQos)
{
  auto node = rclcpp::Node::make_shared("pavo2_conversion_test");
  sensor_msgs::msg::LaserScan::SharedPtr received;
  auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/test_pavo2_scan", rclcpp::QoS(10).reliable(),
    [&received](sensor_msgs::msg::LaserScan::SharedPtr scan) { received = scan; });
  auto publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/test_pavo2_scan", rclcpp::QoS(10).reliable());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(publisher->get_subscription_count(), 1U);
  const std::vector<pavo2::pavo_response_scan_t> points = {
    {4000, 25, 20}, {12000, 500, 30}, {20000, 1500, 40}, {28000, 60000, 50}};
  const rclcpp::Time stamp(123, 456789000, RCL_ROS_TIME);
  publish_scan(publisher, points, stamp, .02, "pavo2s_frame", GetParam(), .1, 100.);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_NE(received, nullptr);
  EXPECT_EQ(received->header.frame_id, "pavo2s_frame");
  EXPECT_EQ(received->header.stamp.sec, 123);
  EXPECT_EQ(received->header.stamp.nanosec, 456789000U);
  EXPECT_NEAR(received->angle_min, -140. * kPi / 180., 1e-6);
  EXPECT_NEAR(received->angle_max, 100. * kPi / 180., 1e-6);
  EXPECT_NEAR(received->angle_increment, 80. * kPi / 180., 1e-6);
  EXPECT_NEAR(received->time_increment, .02 * 80. / 360., 1e-7);
  EXPECT_FLOAT_EQ(received->scan_time, .02F);
  EXPECT_FLOAT_EQ(received->range_min, .1F);
  EXPECT_FLOAT_EQ(received->range_max, 100.F);
  const std::vector<float> ranges = GetParam() ? std::vector<float>{0, 3, 1, 0} : std::vector<float>{0, 1, 3, 0};
  const std::vector<float> intensities = GetParam() ? std::vector<float>{0, 40, 30, 0} : std::vector<float>{0, 30, 40, 0};
  ASSERT_EQ(received->ranges.size(), ranges.size());
  for (size_t i = 0; i < ranges.size(); ++i) {
    EXPECT_FLOAT_EQ(received->ranges[i], ranges[i]);
    EXPECT_FLOAT_EQ(received->intensities[i], intensities[i]);
  }
  EXPECT_EQ(publisher->get_actual_qos().get_rmw_qos_profile().reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(publisher->get_actual_qos().get_rmw_qos_profile().depth, 10U);
}

TEST_P(ScanConversion, RawNoEchoStaysInvalidAfterSdkFiltering)
{
  auto node = rclcpp::Node::make_shared("no_echo_conversion_test");
  auto publisher = node->create_publisher<sensor_msgs::msg::LaserScan>("no_echo_scan", 1);
  sensor_msgs::msg::LaserScan::SharedPtr received;
  auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "no_echo_scan", 1, [&](sensor_msgs::msg::LaserScan::SharedPtr scan) { received = scan; });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!publisher->get_subscription_count() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(publisher->get_subscription_count(), 0u);
  std::vector<pavo2::pavo_response_scan_t> points = {
    {0, 500, 20, true}, {8, 500, 30}, {16, 20, 0}, {24, 500, 4}, {32, 60000, 0}, {40, 500, 0, true}};
  publish_scan(publisher, points, node->now(), .02, "pavo2s_frame", GetParam(), .1, 100.);
  while (!received && std::chrono::steady_clock::now() < deadline) rclcpp::spin_some(node);
  ASSERT_NE(received, nullptr);
  auto values = received->ranges;
  if (GetParam()) std::reverse(values.begin(), values.end());
  EXPECT_EQ(values, (std::vector<float>{0, 1, 0, 1, 0, 0}));
}

INSTANTIATE_TEST_SUITE_P(ExistingDriverBehavior, ScanConversion, ::testing::Bool());
