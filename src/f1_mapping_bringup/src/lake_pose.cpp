#include "lake_pose.h"

namespace f1_mapping {
namespace cc = cartographer::common;
namespace ct = cartographer::transform;
thread_local LakePose* lake_pose = nullptr;

void LakePose::Start(int id, Time now, bool seeded) {
  trajectory = id;
  started = now;
  registered = seeded_shore = false;
  scan_pose = Pose::Identity();
  scan_time.reset();
  fix_time_.reset();
  prev_time_.reset();
  anchor_.reset();
  anchor_time_ = seeded ? std::optional<Time>(now) : std::nullopt;
  AnchorHeading();
}

void LakePose::Scan(Time time, const Pose& pose) {
  scan_time = anchor_time_ = time;
  scan_pose = pose;
  anchor_.reset();
  AnchorHeading();
}

void LakePose::AnchorHeading() {
  if (!anchor_ && anchor_time_) anchor_ = rates_.At(*anchor_time_);
}

void LakePose::Fix(Time time, Eigen::Vector2d xy, double max_age) {
  if (fix_time_ && time <= *fix_time_) return;
  prev_time_ = fix_time_;
  if (prev_time_) prev_xy_ = xy_;
  xy_ = xy;
  fix_time_ = time;
  max_age_ = max_age;
  registered = true;
}

bool LakePose::ScanFresh(Time time) const {
  return scan_time && time >= *scan_time && time - *scan_time <= cc::FromSeconds(.35);
}

bool LakePose::NeedsPrior(Time time) const {
  return registered && (!ScanFresh(time) || seeded_shore);
}

bool LakePose::PositionFresh(Time t) const {
  if (!fix_time_) return false;
  if (t >= *fix_time_) return cc::ToSeconds(t - *fix_time_) <= max_age_;
  return prev_time_ && t >= *prev_time_ && cc::ToSeconds(*fix_time_ - *prev_time_) <= max_age_;
}

std::optional<LakePose::Time> LakePose::HeadingTime(Time now) const {
  auto last = rates_.Latest();
  if (!last || *last > now + cc::FromMilliseconds(100) ||
      now - *last > cc::FromMilliseconds(100)) return std::nullopt;
  return std::min(*last, now);
}

bool LakePose::HeadingValid(Time t) const {
  const auto integral = rates_.At(t);
  return integral && anchor_ && t >= *anchor_time_ && integral->segment == anchor_->segment;
}

LakePose::Pose LakePose::SearchSeed(Time t) const {
  Eigen::Vector2d xy = scan_pose.translation().head<2>();
  if (PositionFresh(t)) xy = xy_;
  double yaw = ct::GetYaw(scan_pose);
  if (HeadingValid(t)) yaw += rates_.At(t)->angle - anchor_->angle;
  return ct::Embed3D(ct::Rigid2d(xy, yaw));
}

std::optional<LakePose::Pose> LakePose::Estimate(Time t) const {
  if (!PositionFresh(t) || !anchor_ || t < *anchor_time_) return std::nullopt;
  const auto integral = rates_.At(t);
  if (!integral || integral->segment != anchor_->segment) return std::nullopt;
  Eigen::Vector2d xy = xy_;
  if (t < *fix_time_) {
    const double fraction = cc::ToSeconds(t - *prev_time_) / cc::ToSeconds(*fix_time_ - *prev_time_);
    xy = prev_xy_ + fraction * (xy_ - prev_xy_);
  }
  return ct::Embed3D(ct::Rigid2d(xy, ct::GetYaw(scan_pose) + integral->angle - anchor_->angle));
}
}  // namespace f1_mapping
