#include "wt102_heading.h"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace f1_mapping {
namespace cc = cartographer::common;
namespace {
const auto kMaxGap = cc::FromMilliseconds(50);
const auto kMaxHold = cc::FromMilliseconds(20);
const auto kHistory = cc::FromSeconds(2.);
}

thread_local const HeadingHistory* wt102_heading = nullptr;

void HeadingHistory::Clear() { samples_.clear(); ++segment_; }

void HeadingHistory::Add(Time stamp, uint64_t stream, double rate) {
  if (!std::isfinite(rate)) {
    Clear();
    return;
  }
  if (stream != stream_ || (!samples_.empty() &&
      (stamp <= samples_.back().stamp || stamp - samples_.back().stamp > kMaxGap))) {
    Clear();
  }
  double angle = 0.;
  if (!samples_.empty()) {
    const auto& previous = samples_.back();
    angle = previous.angle + .5 * (previous.rate + rate) * cc::ToSeconds(stamp - previous.stamp);
  }
  samples_.push_back({stamp, rate, angle});
  stream_ = stream;
  while (samples_.size() > 2 && stamp - samples_[1].stamp > kHistory) samples_.pop_front();
}

double HeadingHistory::AngleAt(Time time) const {
  if (time >= samples_.back().stamp) {
    const auto dt = time - samples_.back().stamp;
    return samples_.back().angle + samples_.back().rate * cc::ToSeconds(dt);
  }
  const auto next = std::upper_bound(samples_.begin(), samples_.end(), time,
      [](Time t, const Sample& sample) { return t < sample.stamp; });
  const auto& prev = *std::prev(next);
  const double dt = cc::ToSeconds(time - prev.stamp);
  const double fraction = dt / cc::ToSeconds(next->stamp - prev.stamp);
  return prev.angle + dt * (prev.rate + .5 * fraction * (next->rate - prev.rate));
}

std::optional<HeadingHistory::Time> HeadingHistory::Latest() const {
  if (samples_.empty()) return std::nullopt;
  return samples_.back().stamp;
}

std::optional<HeadingHistory::Integral> HeadingHistory::At(Time stamp) const {
  if (samples_.size() < 2 || stamp < samples_.front().stamp ||
      stamp > samples_.back().stamp + kMaxHold) return std::nullopt;
  return Integral{segment_, AngleAt(stamp)};
}

std::optional<double> HeadingHistory::Delta(Time start, Time end) const {
  if (samples_.size() < 2 || start < samples_.front().stamp || end < start ||
      end - start > kHistory || end > samples_.back().stamp + kMaxHold) return std::nullopt;
  return AngleAt(end) - AngleAt(start);
}

}  // namespace f1_mapping
