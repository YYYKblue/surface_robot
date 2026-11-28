#pragma once

#include "cartographer/transform/transform.h"
#include "wt102_heading.h"

namespace f1_mapping {

// Every stored pose and antenna fix is in this trajectory's local SLAM frame.
class LakePose {
 public:
  using Time = HeadingHistory::Time;
  using Pose = cartographer::transform::Rigid3d;
  explicit LakePose(const HeadingHistory& rates) : rates_(rates) {}
  void Start(int trajectory, Time now, bool seeded);
  void Scan(Time time, const Pose& pose);
  void AnchorHeading();
  void Fix(Time time, Eigen::Vector2d xy, double max_age);
  bool ScanFresh(Time time) const;
  bool HeadingValid(Time time) const;
  bool PositionFresh(Time time) const;
  std::optional<Time> HeadingTime(Time now) const;
  std::optional<Pose> Estimate(Time time) const;
  bool NeedsPrior(Time time) const;
  Time started{};
  Pose SearchSeed(Time time) const;
  int trajectory = -1;
  bool registered = false;
  std::optional<Time> scan_time;
  Pose scan_pose = Pose::Identity();
  bool seeded_shore = false;

 private:
  const HeadingHistory& rates_;
  std::optional<Time> anchor_time_, fix_time_, prev_time_;
  std::optional<HeadingHistory::Integral> anchor_;
  Eigen::Vector2d xy_, prev_xy_;
  double max_age_ = 1.;
};

extern thread_local LakePose* lake_pose;
}  // namespace f1_mapping
