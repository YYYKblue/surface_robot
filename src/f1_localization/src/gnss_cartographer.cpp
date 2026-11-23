// Fixed-map Cartographer node accepting planar GNSS position and heading.
#include <cmath>
#include <memory>
#include <string>

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/msg/landmark_list.hpp"

namespace mapping = cartographer::mapping;
namespace common = cartographer::common;
using Observation = cartographer_ros_msgs::msg::LandmarkList;

// Services and subscriptions run on a single executor. Only the active
// localization trajectory receives observations; the reference map stays frozen.
class FixedFrameInput {
 public:
  FixedFrameInput(mapping::MapBuilderInterface& builder, double max_gap)
      : builder_(builder), max_gap_(max_gap) {}

  void receive(const Observation& message, const rclcpp::Time& now) {
    if (message.header.frame_id != "base_link" || message.landmarks.size() != 1) return;
    const rclcpp::Time stamp(message.header.stamp);
    const double age = (now.nanoseconds() - stamp.nanoseconds()) * 1e-9;
    if (age < -0.1 || age > 0.5) return;
    const auto& entry = message.landmarks.front();
    if (entry.id.rfind("f1_enu_v1:", 0) != 0) return;
    const auto& p = entry.tracking_from_landmark_transform;
    const auto& q = p.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
        !std::isfinite(p.position.z) || !std::isfinite(norm) || std::abs(norm - 1.) > 1e-5) return;
    auto* graph = static_cast<mapping::PoseGraph*>(builder_.pose_graph());
    const auto time = cartographer_ros::FromRos(stamp);
    for (const auto& [id, state] : graph->GetTrajectoryStates()) {
      if (state != mapping::PoseGraphInterface::TrajectoryState::ACTIVE ||
          !builder_.GetTrajectoryBuilder(id)) continue;
      if (id != trajectory_) {
        trajectory_ = id;
        datum_ = entry.id;
        last_time_.reset();
      }
      if (entry.id != datum_ || (last_time_ && time <= *last_time_)) return;
      if (last_time_ && common::ToSeconds(time - *last_time_) > max_gap_) {
        // Break Cartographer's interpolation across a receiver outage.
        graph->AddFixedFramePoseData(id, {*last_time_ + common::Duration(1), absl::nullopt});
      }
      graph->AddFixedFramePoseData(id, {time, cartographer_ros::ToRigid3d(p).inverse()});
      last_time_ = time;
      return;
    }
  }

 private:
  mapping::MapBuilderInterface& builder_;
  const double max_gap_;
  int trajectory_ = -1;
  std::string datum_;
  absl::optional<common::Time> last_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  cartographer_ros::ScopedRosLogSink log_sink;
  auto ros = rclcpp::Node::make_shared("pool_localization");
  const auto directory = ros->declare_parameter<std::string>("configuration_directory", "");
  const auto basename = ros->declare_parameter<std::string>("configuration_basename", "rtk_localization.lua");
  const auto state = ros->declare_parameter<std::string>("gnss_state_file", "");
  const bool start = ros->declare_parameter("start_trajectory", false);
  auto [options, trajectory] = cartographer_ros::LoadOptions(directory, basename);
  auto* optimizer = options.map_builder_options.mutable_pose_graph_options()->mutable_optimization_problem_options();
  const double position = ros->declare_parameter("gnss_translation_weight", 0.5);
  const double heading = ros->declare_parameter("gnss_rotation_weight", 8.);
  const double rate = ros->declare_parameter("gnss_constraint_rate_hz", 2.);
  if (trajectory.tracking_frame != "base_link" || trajectory.use_landmarks ||
      !(std::isfinite(position) && position > 0 && std::isfinite(heading) && heading > 0 &&
        std::isfinite(rate) && rate > 0)) {
    RCLCPP_ERROR(ros->get_logger(), "Invalid GNSS fixed-frame configuration");
    return 1;
  }
  optimizer->set_fixed_frame_pose_translation_weight(position);
  optimizer->set_fixed_frame_pose_rotation_weight(heading);
  auto builder = mapping::CreateMapBuilder(options.map_builder_options);
  FixedFrameInput input(*builder, 1. / rate + 0.5);
  auto buffer = std::make_shared<tf2_ros::Buffer>(ros->get_clock(), tf2::durationFromSec(10.), ros);
  tf2_ros::TransformListener listener(*buffer);
  cartographer_ros::Node slam(options, std::move(builder), buffer, ros, false);
  slam.LoadState(state, true);
  if (start) slam.StartTrajectoryWithDefaultTopics(trajectory);
  auto sub = ros->create_subscription<Observation>("/f1/rtk/landmarks", 10,
      [&input, &ros](Observation::ConstSharedPtr message) { input.receive(*message, ros->now()); });
  rclcpp::spin(ros);
  slam.FinishAllTrajectories();
  slam.RunFinalOptimization();
  rclcpp::shutdown();
  return 0;
}
