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
#include "f1_interfaces/msg/wt102_sample.hpp"
#include "lake_pose.h"
#include "cartographer_ros/msg_conversion.h"
#include "f1_interfaces/msg/survey_status.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace cm = cartographer::mapping;
namespace ct = cartographer::transform;
namespace cc = cartographer::common;
using Point = geometry_msgs::msg::PointStamped;
using Vector = Eigen::Vector2d;

// Bind the local prediction to the lifetime of the one active ROS trajectory.
// StartTrajectory supplies the operator seed as global_from_local at t=0.
class SurveyBuilder : public cm::MapBuilder {
 public:
  SurveyBuilder(const cm::proto::MapBuilderOptions& options, f1_mapping::LakePose& lake,
                rclcpp::Clock::SharedPtr clock)
      : cm::MapBuilder(options), lake_(lake), clock_(std::move(clock)) {}
  int AddTrajectoryBuilder(const std::set<SensorId>& sensors,
                           const cm::proto::TrajectoryBuilderOptions& options,
                           LocalSlamResultCallback callback) override {
    const int id = cm::MapBuilder::AddTrajectoryBuilder(sensors, options, std::move(callback));
    lake_.Start(id, cartographer_ros::FromRos(clock_->now()), options.has_initial_trajectory_pose());
    return id;
  }
  void FinishTrajectory(int id) override {
    cm::MapBuilder::FinishTrajectory(id);
    if (lake_.trajectory == id) lake_.Start(-1, cartographer_ros::FromRos(clock_->now()), false);
  }
 private:
  f1_mapping::LakePose& lake_;
  rclcpp::Clock::SharedPtr clock_;
};

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
    auto states = graph->GetTrajectoryStates();
    // A manually seeded trajectory can cross open water before its first scan
    // node is inserted into the pose graph.
    if (f1_mapping::lake_pose && f1_mapping::lake_pose->trajectory >= 0)
      states[f1_mapping::lake_pose->trajectory] = cm::PoseGraphInterface::TrajectoryState::ACTIVE;
    for (const auto& [id, state] : states) {
      if (auto* lake = f1_mapping::lake_pose; lake && (id != lake->trajectory || time < lake->started))
        continue;
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
      if (auto* lake = f1_mapping::lake_pose; lake && lake->trajectory == id) {
        ct::Rigid3d map_from_fix = ct::Rigid3d::Identity();
        if (!registered_map) {
          map_from_fix = map_from_enu_;
          const auto data = graph->GetTrajectoryData();
          auto it = data.find(id);
          if (it != data.end() && it->second.fixed_frame_origin_in_map)
            map_from_fix = *it->second.fixed_frame_origin_in_map;
        }
        const auto local_from_fix = graph->GetLocalToGlobalTransform(id).inverse() * map_from_fix;
        lake->Fix(time, (local_from_fix * Eigen::Vector3d(xy.x(), xy.y(), 0.)).head<2>(), max_gap_);
      }
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
    if (auto* lake = f1_mapping::lake_pose) {
      const auto time = cartographer_ros::FromRos(stamp);
      if (!lake->scan_time || std::abs(cc::ToSeconds(time - *lake->scan_time)) > .35) return false;
    }
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
    map_from_enu_ = map_from_enu;
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
  ct::Rigid3d map_from_enu_ = ct::Rigid3d::Identity();
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
  const bool use_gnss = ros->declare_parameter("use_rtk", true);
  const bool use_heading = ros->declare_parameter("use_wt102_heading", false);
  if (use_gnss && (trajectory.tracking_frame != "gnss_ant1" || trajectory.use_landmarks || trajectory.use_nav_sat))
    throw std::invalid_argument("Position-only fusion requires independent ANT1 tracking");
  auto* optimizer = options.map_builder_options.mutable_pose_graph_options()->mutable_optimization_problem_options();
  optimizer->set_fixed_frame_pose_translation_weight(weight);
  optimizer->set_fixed_frame_pose_rotation_weight(0.);
  f1_mapping::HeadingHistory heading;
  f1_mapping::LakePose lake(heading);
  using Sample = f1_interfaces::msg::Wt102Sample;
  rclcpp::Subscription<Sample>::SharedPtr heading_sub;
  if (use_heading) {
    if (!F1_WT102_SUPPORTED)
      throw std::invalid_argument("WT102 prediction requires the static ROS Humble cartographer 2.0.9004 library");
    const auto& local = trajectory.trajectory_builder_options.trajectory_builder_2d_options();
    if (!options.map_builder_options.use_trajectory_builder_2d() || local.use_imu_data() ||
        trajectory.use_odometry) {
      throw std::invalid_argument("WT102 prediction requires a planar trajectory");
    }
    const double bias = ros->declare_parameter("wt102_bias_rad_s", 0.);
    if (!std::isfinite(bias)) throw std::invalid_argument("Invalid WT102 Z bias");
    f1_mapping::wt102_heading = &heading;
    heading_sub = ros->create_subscription<Sample>("/f1/wt102/sample", rclcpp::SensorDataQoS().keep_last(100),
        [&heading, &lake, bias, ros](Sample::ConstSharedPtr sample) {
          if (sample->kind != Sample::YAW_RATE || sample->header.frame_id != "wt102_link") return;
          const auto& header_stamp = sample->header.stamp;
          if (header_stamp.sec < 0 || header_stamp.nanosec >= 1000000000u ||
              (header_stamp.sec == 0 && header_stamp.nanosec == 0)) return;
          const rclcpp::Time stamp(header_stamp);
          const auto age = (ros->now() - stamp).seconds();
          if (age < -.1 || age > .2) return;
          heading.Add(cartographer_ros::FromRos(stamp), sample->stream_id, sample->value - bias);
          lake.AnchorHeading();
        });
  }
  const bool survey = use_gnss && use_heading;
  std::unique_ptr<cm::MapBuilderInterface> builder;
  if (survey) {
    options.publish_to_tf = false;
    f1_mapping::lake_pose = &lake;
    builder = std::make_unique<SurveyBuilder>(options.map_builder_options, lake, ros->get_clock());
  } else {
    builder = cm::CreateMapBuilder(options.map_builder_options);
  }
  auto* graph = builder->pose_graph();
  auto buffer = std::make_shared<tf2_ros::Buffer>(ros->get_clock(), tf2::durationFromSec(10.), ros);
  tf2_ros::TransformListener listener(*buffer);
  PositionInput input(*builder, *buffer, rate, spread, tolerance);
  cartographer_ros::Node slam(options, std::move(builder), buffer, ros, false);
  if (!state.empty()) slam.LoadState(state, true);
  if (start) slam.StartTrajectoryWithDefaultTopics(trajectory);
  // Give TF time to bracket each sample. Never use the latest pose in place
  // of a missing transform at the measurement time.
  std::deque<Point::ConstSharedPtr> pending;
  rclcpp::Subscription<Point>::SharedPtr sub;
  if (use_gnss) sub = ros->create_subscription<Point>("/f1/rtk/position_constraint", 10,
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
      if ((!survey || !lake.registered) && pending.front()->header.frame_id != "map" && age >= -.1 && age < .45 &&
          !buffer->canTransform("map", "gnss_ant1", stamp, tf2::durationFromSec(0.), &tf_error)) break;
      input.receive(*pending.front(), now);
      pending.pop_front();
    }
  });
  using Status = f1_interfaces::msg::SurveyStatus;
  auto tf = std::make_unique<tf2_ros::TransformBroadcaster>(ros);
  auto status = ros->create_publisher<Status>("/f1/survey/status", 1);
  std::optional<cc::Time> last_tf;
  int last_id = -1;
  auto report = ros->create_wall_timer(std::chrono::milliseconds(50), [&] {
    if (!survey) return;
    const auto stamp = ros->now();
    const auto now = cartographer_ros::FromRos(stamp);
    Status msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.trajectory_id = lake.trajectory;
    msg.registered = lake.registered;
    msg.position_fresh = lake.PositionFresh(now);
    const auto gyro = lake.HeadingTime(now);
    msg.heading_valid = gyro && lake.HeadingValid(*gyro);
    auto pose_time = now;
    std::optional<ct::Rigid3d> pose;
    if (lake.ScanFresh(now)) {
      pose = lake.scan_pose;
      pose_time = *lake.scan_time;
      msg.source = lake.seeded_shore ? "new_shore" : "lidar";
    } else if (gyro && msg.position_fresh && (pose = lake.Estimate(*gyro))) {
      pose_time = *gyro;
      msg.source = "gnss_imu";
    } else {
      msg.source = "unavailable";
    }
    if (lake.trajectory != last_id) last_tf.reset();
    last_id = lake.trajectory;
    if (pose && lake.trajectory >= 0 && (!last_tf || pose_time > *last_tf)) {
      try {
        const auto tracking_from_base = cartographer_ros::ToRigid3d(
            buffer->lookupTransform("gnss_ant1", "base_link", tf2::TimePointZero));
        geometry_msgs::msg::TransformStamped global, local;
        global.header.stamp = local.header.stamp = cartographer_ros::ToRos(pose_time);
        global.header.frame_id = "map";
        global.child_frame_id = local.header.frame_id = "odom";
        local.child_frame_id = "base_link";
        global.transform = cartographer_ros::ToGeometryMsgTransform(graph->GetLocalToGlobalTransform(lake.trajectory));
        local.transform = cartographer_ros::ToGeometryMsgTransform(*pose * tracking_from_base);
        tf->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{global, local});
        last_tf = pose_time;
      } catch (const tf2::TransformException&) {
        msg.source = "unavailable";
      }
    }
    status->publish(msg);
  });
  rclcpp::spin(ros);
  slam.FinishAllTrajectories();
  slam.RunFinalOptimization();
  rclcpp::shutdown();
}
