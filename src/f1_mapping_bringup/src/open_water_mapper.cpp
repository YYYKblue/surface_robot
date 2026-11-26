// Georeferenced survey grid. Pose is supplied by the fixed RTK pair even when
// there are no laser hits. Both /map and the saved submap use the same cells.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <system_error>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/msg/landmark_list.hpp"
#include "cartographer_ros_msgs/srv/write_state.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace cm = cartographer::mapping;
namespace ct = cartographer::transform;
using Observation = cartographer_ros_msgs::msg::LandmarkList;
using Scan = sensor_msgs::msg::LaserScan;
using Save = cartographer_ros_msgs::srv::WriteState;
using Grid = nav_msgs::msg::OccupancyGrid;
using Pose = ct::Rigid3d;
using Cell = Eigen::Array2i;

class WaterMapper : public rclcpp::Node {
 public:
  WaterMapper() : Node("open_water_mapper"), broadcaster_(*this) {
    resolution_ = declare_parameter("resolution", .05);
    radius_ = declare_parameter("no_echo_radius_m", 3.);
    max_range_ = declare_parameter("hit_range_m", 30.);
    if (!std::isfinite(resolution_) || resolution_ < .02 || resolution_ > 1 ||
        !std::isfinite(radius_) || radius_ <= 0 || radius_ > 30 ||
        !std::isfinite(max_range_) || max_range_ < radius_ || max_range_ > 100)
      throw std::invalid_argument("invalid open water grid dimensions");
    const auto mount = declare_parameter<std::vector<double>>("mount", {0., 0., 0.});
    if (mount.size() != 3 || !std::all_of(mount.begin(), mount.end(), [](double v) { return std::isfinite(v); }))
      throw std::invalid_argument("invalid lidar mounting transform");
    mount_ = ct::Rigid2d({mount[0], mount[1]}, mount[2]);
    auto directory = declare_parameter<std::string>("configuration_directory", "");
    options_ = std::get<1>(cartographer_ros::LoadOptions(directory, "rtk_mapping.lua")).trajectory_builder_options;
    grid_ = std::make_unique<cm::ProbabilityGrid>(
        cm::MapLimits(resolution_, {32 * resolution_, 32 * resolution_}, {64, 64}), &tables_);
    map_pub_ = create_publisher<Grid>("/map", rclcpp::QoS(1).transient_local());
    scan_sub_ = create_subscription<Scan>("water_scan", rclcpp::SensorDataQoS().keep_last(2),
      [this](Scan::ConstSharedPtr s) {
        previous_ = std::move(scan_);
        scan_ = std::move(s);
      });
    pose_sub_ = create_subscription<Observation>("/f1/rtk/landmarks", 10,
      [this](Observation::ConstSharedPtr m) { observe(*m); });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      if (dirty_) publish_map();
    });
    save_ = create_service<Save>(
      "/write_state", [this](Save::Request::SharedPtr req, Save::Response::SharedPtr resp) {
      try {
        if (poses_.empty()) throw std::runtime_error("no synchronized fixed RTK and scan observations");
        if (std::filesystem::path(req->filename).extension() != ".pbstream")
          throw std::runtime_error("expected .pbstream filename");
        save_snapshot(req->filename);
        resp->status.code = 0;
      } catch (const std::exception& e) {
        resp->status.code = 9;
        resp->status.message = e.what();
      }
    });
  }
 private:
  struct Beam {
    Eigen::Vector2f endpoint;
    bool inferred;
    bool hit;
  };

  // Grow only the sides the survey reaches. Cell indices shift by an integer
  // offset so extending a long, narrow route never moves an existing obstacle.
  bool reserve_grid(const Eigen::AlignedBox2f& bounds) {
    const auto& limits = grid_->limits();
    const Cell first = limits.GetCellIndex(bounds.max()) - 1;
    const Cell last = limits.GetCellIndex(bounds.min()) + 1;
    if (limits.Contains(first) && limits.Contains(last)) return true;

    const auto& old = limits.cell_limits();
    const Cell size(old.num_x_cells, old.num_y_cells);
    const auto blocks = [](int count) { return count > 0 ? ((count + 127) / 128) * 128 : 0; };
    const Cell before = (-first).unaryExpr(blocks);
    const Cell after = (last + 1 - size).unaryExpr(blocks);
    const Cell expanded = size + before + after;
    if (expanded.maxCoeff() > 16384 || int64_t(expanded.x()) * expanded.y() > 16777216) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Survey exceeds grid capacity; save this section");
      return false;
    }
    const Eigen::Vector2d upper = limits.max() + resolution_ * Eigen::Vector2d(before.y(), before.x());
    auto larger = std::make_unique<cm::ProbabilityGrid>(
        cm::MapLimits(resolution_, upper, {expanded.x(), expanded.y()}), &tables_);
    Cell offset;
    cm::CellLimits known;
    grid_->ComputeCroppedLimits(&offset, &known);
    for (int y = 0; y < known.num_y_cells; ++y) {
      for (int x = 0; x < known.num_x_cells; ++x) {
        const Cell index = offset + Cell(x, y);
        if (grid_->IsKnown(index)) larger->SetProbability(index + before, grid_->GetProbability(index));
      }
    }
    grid_ = std::move(larger);
    return true;
  }

  void trace_ray(Cell start, const Cell& end, bool inferred) {
    const int dx = std::abs(end.x() - start.x());
    const int dy = -std::abs(end.y() - start.y());
    const int sx = start.x() < end.x() ? 1 : -1;
    const int sy = start.y() < end.y() ? 1 : -1;
    int error = dx + dy;
    for (;;) {
      if (grid_->IsKnown(start)) {
        if (inferred && grid_->GetProbability(start) > .5) break;
      } else {
        grid_->SetProbability(start, .1f);
      }
      if ((start == end).all()) break;
      const int twice = 2 * error;
      if (twice >= dy) {
        error += dy;
        start.x() += sx;
      }
      if (twice <= dx) {
        error += dx;
        start.y() += sy;
      }
    }
  }

  std::vector<Beam> project_scan(const Scan& scan, const ct::Rigid2d& origin) const {
    std::vector<Beam> beams;
    if (scan.header.frame_id != "pavo2s_frame" || scan.ranges.size() < 2 ||
        !std::isfinite(scan.angle_increment) || scan.angle_increment <= 0 ||
        !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_max) ||
        !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
        scan.range_min <= 0 || scan.range_max < scan.range_min ||
        std::abs(scan.angle_min + (scan.ranges.size() - 1) * scan.angle_increment - scan.angle_max) > .01)
      return beams;
    const double interval = previous_ ?
        (rclcpp::Time(scan.header.stamp) - rclcpp::Time(previous_->header.stamp)).seconds() : 0.;
    const bool repeated_scan = interval > 0 && interval <= .1 &&
        previous_->header.frame_id == scan.header.frame_id &&
        previous_->ranges.size() == scan.ranges.size() &&
        previous_->angle_min == scan.angle_min && previous_->angle_increment == scan.angle_increment;
    beams.reserve(scan.ranges.size());
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double range = scan.ranges[i];
      const bool inferred = range == std::numeric_limits<float>::infinity();
      if (inferred && !(repeated_scan && previous_->ranges[i] == range)) continue;
      if (!inferred && (!std::isfinite(range) || range < scan.range_min || range > scan.range_max)) continue;
      const double angle = scan.angle_min + i * scan.angle_increment;
      const double length = inferred ? std::min(radius_, double(scan.range_max)) : std::min(range, max_range_);
      const Eigen::Vector2d endpoint = origin * (length * Eigen::Vector2d(std::cos(angle), std::sin(angle)));
      beams.push_back({endpoint.cast<float>(), inferred, !inferred && range <= max_range_});
    }
    return beams;
  }

  void observe(const Observation& obs) {
    if (!scan_ || obs.landmarks.size() != 1 || obs.header.frame_id != "base_link") return;
    const auto& entry = obs.landmarks.front();
    if (entry.id.rfind("f1_enu_v1:", 0) != 0 || (!datum_.empty() && entry.id != datum_)) return;
    const rclcpp::Time stamp(obs.header.stamp), scan_stamp(scan_->header.stamp);
    const double age = (now() - stamp).seconds();
    if (age < -.1 || age > .5 || std::abs((scan_stamp - stamp).seconds()) > .12 ||
        stamp.nanoseconds() <= last_stamp_ || scan_stamp.nanoseconds() <= last_scan_stamp_) return;

    const Pose pose = cartographer_ros::ToRigid3d(entry.tracking_from_landmark_transform).inverse();
    if (!pose.IsValid() || !pose.translation().allFinite()) return;
    const auto origin = ct::Project2D(pose) * mount_;
    const auto beams = project_scan(*scan_, origin);
    if (beams.empty()) return;
    Eigen::AlignedBox2f bounds;
    bounds.extend(origin.translation().cast<float>());
    for (const Beam& beam : beams) bounds.extend(beam.endpoint);
    if (!reserve_grid(bounds)) return;
    const auto& limits = grid_->limits();
    const Cell centre = limits.GetCellIndex(origin.translation().cast<float>());
    // Mark every hit before tracing, including hits in cells previously drawn free.
    for (const Beam& beam : beams) {
      if (!beam.hit) continue;
      const Cell end = limits.GetCellIndex(beam.endpoint);
      if (!grid_->IsKnown(end)) grid_->SetProbability(end, .9f);
      else if (grid_->GetProbability(end) < .65) grid_->ApplyLookupTable(end, hit_table_);
    }
    grid_->FinishUpdate();
    Cell last = Cell::Constant(-1);
    bool last_inferred = false;
    for (const Beam& beam : beams) {
      const Cell end = limits.GetCellIndex(beam.endpoint);
      if ((end != last).any() || last_inferred != beam.inferred) {
        trace_ray(centre, end, beam.inferred);
        last = end;
        last_inferred = beam.inferred;
      }
    }

    poses_.emplace_back(cartographer_ros::FromRos(stamp), pose);
    datum_ = entry.id;
    last_stamp_ = stamp.nanoseconds();
    last_scan_stamp_ = scan_stamp.nanoseconds();
    dirty_ = true;
    geometry_msgs::msg::TransformStamped tf;
    tf.header = obs.header;
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";
    tf.transform = cartographer_ros::ToGeometryMsgTransform(pose);
    broadcaster_.sendTransform(tf);
  }

  void publish_map() {
    Cell offset;
    cm::CellLimits dims;
    grid_->ComputeCroppedLimits(&offset, &dims);
    Grid msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.info.resolution = resolution_;
    msg.info.width = dims.num_y_cells;
    msg.info.height = dims.num_x_cells;
    const Eigen::Vector2d lower = grid_->limits().max() - resolution_ * Eigen::Vector2d(
        offset.y() + dims.num_y_cells, offset.x() + dims.num_x_cells);
    msg.info.origin.position.x = lower.x();
    msg.info.origin.position.y = lower.y();
    msg.info.origin.orientation.w = 1;
    msg.data.resize(size_t(msg.info.width) * msg.info.height, -1);
    for (int y = 0; y < dims.num_x_cells; ++y) {
      for (int x = 0; x < dims.num_y_cells; ++x) {
        const Cell index = offset + Cell(dims.num_x_cells - 1 - y, dims.num_y_cells - 1 - x);
        if (grid_->IsKnown(index)) {
          msg.data[y * msg.info.width + x] = grid_->GetProbability(index) > .5 ? 100 : 0;
        }
      }
    }
    map_pub_->publish(msg);
    dirty_ = false;
  }

  void save_snapshot(const std::string& filename) {
    // App recovery treats a non-empty .pbstream as a saved state. Make it
    // visible only after all records and the close operation succeeded.
    std::string temporary = filename + ".XXXXXX";
    const int fd = ::mkstemp(temporary.data());
    if (fd < 0) throw std::system_error(errno, std::generic_category(), "create snapshot");
    ::close(fd);
    try {
      write_state(temporary);
      std::filesystem::rename(temporary, filename);
    } catch (...) {
      std::remove(temporary.c_str());
      throw;
    }
  }

  void write_state(const std::string& filename) {
    namespace proto = cm::proto;
    cartographer::io::ProtoStreamWriter writer(filename);
    proto::SerializationHeader header;
    header.set_format_version(2);
    writer.WriteProto(header);

    proto::SerializedData data;
    auto* graph = data.mutable_pose_graph();
    auto* trajectory = graph->add_trajectory();
    trajectory->set_trajectory_id(0);
    const auto identity = ct::ToProto(Pose::Identity());
    auto* submap = trajectory->add_submap();
    submap->set_submap_index(0);
    *submap->mutable_pose() = identity;
    auto* datum = graph->add_landmark_poses();
    datum->set_landmark_id(datum_);
    *datum->mutable_global_pose() = identity;
    for (size_t i = 0; i < poses_.size(); ++i) {
      const auto& [stamp, pose] = poses_[i];
      auto* node = trajectory->add_node();
      node->set_node_index(i);
      node->set_timestamp(cartographer::common::ToUniversal(stamp));
      *node->mutable_pose() = ct::ToProto(pose);
      auto* constraint = graph->add_constraint();
      constraint->mutable_node_id()->set_node_index(i);
      constraint->mutable_submap_id()->set_submap_index(0);
      *constraint->mutable_relative_pose() = ct::ToProto(pose);
      constraint->set_translation_weight(1);
      constraint->set_rotation_weight(1);
    }
    writer.WriteProto(data);

    data.Clear();
    auto* options = data.mutable_all_trajectory_builder_options()->add_options_with_sensor_ids();
    *options->mutable_trajectory_builder_options() = options_;
    writer.WriteProto(data);

    data.Clear();
    cm::Submap2D snapshot(Eigen::Vector2f::Zero(), grid_->ComputeCroppedGrid(), &tables_);
    auto* serialized = data.mutable_submap();
    *serialized = snapshot.ToProto(true);
    serialized->mutable_submap_id()->set_submap_index(0);
    serialized->mutable_submap_2d()->set_finished(true);
    serialized->mutable_submap_2d()->set_num_range_data(poses_.size());
    writer.WriteProto(data);

    for (size_t i = 0; i < poses_.size(); ++i) {
      data.Clear();
      auto* node = data.mutable_node();
      node->mutable_node_id()->set_node_index(i);
      cm::TrajectoryNode::Data record{};
      record.time = poses_[i].first;
      record.local_pose = poses_[i].second;
      record.gravity_alignment = Eigen::Quaterniond::Identity();
      *node->mutable_node_data() = cm::ToProto(record);
      writer.WriteProto(data);
    }
    if (!writer.Close()) throw std::runtime_error("could not write survey snapshot");
  }

  cm::ValueConversionTables tables_;
  std::unique_ptr<cm::ProbabilityGrid> grid_;
  const std::vector<uint16_t> hit_table_ = cm::ComputeLookupTableToApplyCorrespondenceCostOdds(1000.f);
  std::vector<std::pair<cartographer::common::Time, Pose>> poses_;
  cm::proto::TrajectoryBuilderOptions options_;
  ct::Rigid2d mount_;
  std::string datum_;
  double resolution_, radius_, max_range_;
  int64_t last_stamp_ = 0, last_scan_stamp_ = 0;
  bool dirty_ = false;
  Scan::ConstSharedPtr previous_, scan_;
  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Publisher<Grid>::SharedPtr map_pub_;
  rclcpp::Subscription<Scan>::SharedPtr scan_sub_;
  rclcpp::Subscription<Observation>::SharedPtr pose_sub_;
  rclcpp::Service<Save>::SharedPtr save_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaterMapper>());
  rclcpp::shutdown();
}
