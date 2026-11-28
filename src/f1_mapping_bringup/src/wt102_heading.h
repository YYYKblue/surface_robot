#pragma once

#include <cstdint>
#include <deque>
#include <optional>

#include "cartographer/common/time.h"

namespace f1_mapping {

// Integrated body-Z rate. Every accepted interval belongs to one continuous
// stream. The last rate may be held briefly while waiting for the next sample.
class HeadingHistory {
 public:
  using Time = cartographer::common::Time;
  struct Integral { uint64_t segment; double angle; };
  std::optional<Integral> At(Time stamp) const;
  std::optional<Time> Latest() const;
  void Clear();
  void Add(Time stamp, uint64_t stream, double rate);
  std::optional<double> Delta(Time start, Time end) const;

 private:
  struct Sample {
    Time stamp;
    double rate;
    double angle;
  };
  double AngleAt(Time time) const;
  std::deque<Sample> samples_;
  uint64_t stream_ = 0, segment_ = 0;
};

// This executable spins a single ROS executor. Rate and scan callbacks as
// well as ROS pose publication run on it. Each extrapolator uses its own last
// matched pose as anchor; the history contains only physical measurements.
extern thread_local const HeadingHistory* wt102_heading;

}  // namespace f1_mapping
