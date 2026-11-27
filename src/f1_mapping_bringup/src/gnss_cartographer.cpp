// The tracking origin is ANT1, with axes parallel to base_link. The scan
// matcher still publishes base_link. A GNSS sample measures translation only.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/time_conversion.h"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace cm = cartographer::mapping;
namespace ct = cartographer::transform;
namespace cc = cartographer::common;
using Point = geometry_msgs::msg::PointStamped;
using Vector = Eigen::Vector2d;

class PositionInput {
 public:
  PositionInput(cm::MapBuilderInterface& builder, tf2_ros::Buffer& tf,
                double rate, double baseline, double tolerance)
      : builder_(builder), tf_(tf), max_gap_(1./rate + .5),
        baseline_(baseline), tolerance_(tolerance) {}

  void receive(const Point& msg, const rclcpp::Time& now) {
    const rclcpp::Time stamp(msg.header.stamp);
    const double age = (now - stamp).seconds();
    const Vector xy(msg.point.x, msg.point.y);
    const bool registered_map = msg.header.frame_id == "map";
    if ((!registered_map && msg.header.frame_id.rfind("f1_gga_v2:", 0) != 0) ||
        !xy.allFinite() || xy.cwiseAbs().maxCoeff() > 1e6 || age < -.1 || age > .5) return;
    auto* graph = static_cast<cm::PoseGraph*>(builder_.pose_graph());
    const auto time = cartographer_ros::FromRos(stamp);
    for (const auto& [id, state] : graph->GetTrajectoryStates()) {
      if (state != cm::PoseGraphInterface::TrajectoryState::ACTIVE ||
          !builder_.GetTrajectoryBuilder(id)) continue;
      if (trajectory_ != id) {
        trajectory_ = id;
        frame_ = msg.header.frame_id;
        last_.reset();
        pairs_.clear();
        aligned_ = false;
      }
      if (frame_ != msg.header.frame_id || (last_ && time <= *last_)) return;

      // For registered maps the manager projects GGA into map coordinates.
      // Initialize the active trajectory's fixed frame at the saved registration.
      if (registered_map && !aligned_) {
        set_origin(*graph, id, ct::Rigid3d::Identity());
        aligned_ = true;
      }
      if (!registered_map && !aligned_ && !align(*graph, id, xy, stamp)) return;
      if (last_ && cc::ToSeconds(time - *last_) > max_gap_)
        graph->AddFixedFramePoseData(id, {*last_ + cc::Duration(1), absl::nullopt});
      graph->AddFixedFramePoseData(id, {time, ct::Rigid3d::Translation({xy.x(), xy.y(), 0.})});
      last_ = time;
      return;
    }
  }

 private:
  static void set_origin(cm::PoseGraph& graph, int id, const ct::Rigid3d& map_from_fixed) {
    cm::proto::TrajectoryData datum;
    datum.set_trajectory_id(id);
    datum.set_gravity_constant(9.8);
    datum.mutable_imu_calibration()->set_w(1.);
    *datum.mutable_fixed_frame_origin_in_map() = ct::ToProto(map_from_fixed);
    graph.SetTrajectoryDataFromProto(datum);
  }

  bool align(cm::PoseGraph& graph, int id, const Vector& enu, const rclcpp::Time& stamp) {
    // Scan poses and GGA must describe the same antenna at the same instant.
    geometry_msgs::msg::TransformStamped pose;
    try {
      pose = tf_.lookupTransform("map", "gnss_ant1", stamp);
    } catch (const tf2::TransformException&) {
      return false;
    }
    const auto& p = pose.transform.translation;
    pairs_.emplace_back(enu, Vector(p.x, p.y));
    if (pairs_.size() > 240) pairs_.pop_front();
    if (pairs_.size() < 6) return false;
    Vector a = Vector::Zero(), b = Vector::Zero();
    for (const auto& pair : pairs_) {
      a += pair.first;
      b += pair.second;
    }
    a /= pairs_.size();
    b /= pairs_.size();
    double cross = 0., dot = 0., enu_spread = 0., map_spread = 0.;
    for (const auto& pair : pairs_) {
      const Vector u = pair.first - a, v = pair.second - b;
      cross += u.x()*v.y() - u.y()*v.x();
      dot += u.dot(v);
      enu_spread += u.squaredNorm();
      map_spread += v.squaredNorm();
    }
    // Require a measurable track on BOTH sensors. Static GGA scatter alone
    // cannot establish the orientation of the map's geographic reference.
    const double min_spread = pairs_.size() * baseline_ * baseline_;
    if (std::min(enu_spread, map_spread) < min_spread || std::hypot(dot, cross) < min_spread*.8)
      return false;
    const Eigen::Rotation2Dd rotation(std::atan2(cross, dot));
    const Vector offset = b - rotation*a;
    double error = 0.;
    for (const auto& pair : pairs_) error += (rotation*pair.first + offset - pair.second).squaredNorm();
    if (error > pairs_.size()*tolerance_*tolerance_) return false;
    const auto map_from_enu = ct::Embed3D(ct::Rigid2d(offset, rotation));
    set_origin(graph, id, map_from_enu);
    // The marker carries the WGS84 origin. read_gnss_reference uses the
    // trajectory's optimized fixed_frame_origin_in_map from the same snapshot.
    graph.SetLandmarkPose(frame_, map_from_enu, true);
    pairs_.clear();
    aligned_ = true;
    return true;
  }

  cm::MapBuilderInterface& builder_;
  tf2_ros::Buffer& tf_;
  double max_gap_, baseline_, tolerance_;
  int trajectory_ = -1;
  std::string frame_;
  absl::optional<cc::Time> last_;
  std::deque<std::pair<Vector, Vector>> pairs_;
  bool aligned_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  cartographer_ros::ScopedRosLogSink sink;
  auto ros = rclcpp::Node::make_shared("gnss_cartographer");
  const auto directory = ros->declare_parameter<std::string>("configuration_directory", "");
  const auto basename = ros->declare_parameter<std::string>("configuration_basename", "rtk_mapping.lua");
  const auto state = ros->declare_parameter<std::string>("gnss_state_file", "");
  const bool start = ros->declare_parameter("start_trajectory", true);
  auto [options, trajectory] = cartographer_ros::LoadOptions(directory, basename);
  const double weight = ros->declare_parameter("gnss_translation_weight", .5);
  const double rate = ros->declare_parameter("gnss_constraint_rate_hz", 2.);
  const double spread = ros->declare_parameter("gnss_alignment_rms_spread_m", 1.);
  const double tolerance = ros->declare_parameter("gnss_alignment_max_error_m", .45);
  for (double v : {weight, rate, spread, tolerance})
    if (!std::isfinite(v) || v <= 0.) throw std::invalid_argument("Invalid GNSS position configuration");
  if (trajectory.tracking_frame != "gnss_ant1" || trajectory.use_landmarks || trajectory.use_nav_sat)
    throw std::invalid_argument("Position-only fusion requires independent ANT1 tracking");
  auto* optimizer = options.map_builder_options.mutable_pose_graph_options()->mutable_optimization_problem_options();
  optimizer->set_fixed_frame_pose_translation_weight(weight);
  optimizer->set_fixed_frame_pose_rotation_weight(0.);
  auto builder = cm::CreateMapBuilder(options.map_builder_options);
  auto buffer = std::make_shared<tf2_ros::Buffer>(ros->get_clock(), tf2::durationFromSec(10.), ros);
  tf2_ros::TransformListener listener(*buffer);
  PositionInput input(*builder, *buffer, rate, spread, tolerance);
  cartographer_ros::Node slam(options, std::move(builder), buffer, ros, false);
  if (!state.empty()) slam.LoadState(state, true);
  if (start) slam.StartTrajectoryWithDefaultTopics(trajectory);
  // Give TF time to bracket each sample. Never use the latest pose in place
  // of a missing transform at the measurement time.
  std::deque<Point::ConstSharedPtr> pending;
  auto sub = ros->create_subscription<Point>("/f1/rtk/position_constraint", 10,
      [&pending](Point::ConstSharedPtr msg) {
        pending.push_back(msg);
        if (pending.size() > 32) pending.pop_front();
      });
  auto timer = ros->create_wall_timer(std::chrono::milliseconds(25), [&] {
    const auto now = ros->now();
    while (!pending.empty()) {
      const rclcpp::Time stamp(pending.front()->header.stamp);
      const double age = (now - stamp).seconds();
      std::string tf_error;
      if (pending.front()->header.frame_id != "map" && age >= -.1 && age < .45 &&
          !buffer->canTransform("map", "gnss_ant1", stamp, tf2::durationFromSec(0.), &tf_error)) break;
      input.receive(*pending.front(), now);
      pending.pop_front();
    }
  });
  rclcpp::spin(ros);
  slam.FinishAllTrajectories();
  slam.RunFinalOptimization();
  rclcpp::shutdown();
}
