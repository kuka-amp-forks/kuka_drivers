// Copyright 2026 KUKA Hungaria Kft.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KUKA_RSI_DRIVER__VELOCITY_ESTIMATOR_HPP_
#define KUKA_RSI_DRIVER__VELOCITY_ESTIMATOR_HPP_

#include <cmath>
#include <cstdint>
#include <vector>

namespace kuka_rsi_driver
{

// Joint velocities differentiated from the positions of consecutive RSI packets, timed by the
// KRC's IPOC (milliseconds), then smoothed by a second-order Butterworth low-pass.
class VelocityEstimator
{
public:
  // A cutoff of zero or at or above Nyquist disables the low-pass.
  void Configure(size_t joints, double cutoff_hz, double sample_period_s)
  {
    filtered_ = cutoff_hz > 0.0 && cutoff_hz < 0.5 / sample_period_s;
    if (filtered_)
    {
      // Bilinear transform with prewarping at the cutoff, for the nominal period only: a sample
      // after a dropped packet still counts as one filter step.
      const double k = std::tan(M_PI * cutoff_hz * sample_period_s);
      const double norm = 1.0 / (1.0 + M_SQRT2 * k + k * k);
      b0_ = k * k * norm;
      b1_ = 2.0 * b0_;
      b2_ = b0_;
      a1_ = 2.0 * (k * k - 1.0) * norm;
      a2_ = (1.0 - M_SQRT2 * k + k * k) * norm;
    }
    joints_.assign(joints, Joint{});
    Reset();
  }

  void Reset() { primed_ = false; }

  // Returns false when the sample repeats the last IPOC and velocities is left as it was.
  bool Update(
    const std::vector<double> & positions, uint64_t ipoc, std::vector<double> & velocities)
  {
    if (primed_ && ipoc == last_ipoc_)
    {
      return false;
    }
    const double dt = static_cast<double>(ipoc - last_ipoc_) * kSecondsPerIpoc;
    for (size_t i = 0; i < joints_.size(); i++)
    {
      auto & joint = joints_[i];
      if (!primed_)
      {
        joint = Joint{};
        joint.position = positions[i];
        velocities[i] = 0.0;
        continue;
      }
      const double raw = (positions[i] - joint.position) / dt;
      joint.position = positions[i];
      velocities[i] = filtered_ ? Filter(joint, raw) : raw;
    }
    last_ipoc_ = ipoc;
    primed_ = true;
    return true;
  }

private:
  struct Joint
  {
    double position = 0.0;
    double x1 = 0.0;
    double x2 = 0.0;
    double y1 = 0.0;
    double y2 = 0.0;
  };

  double Filter(Joint & joint, double x)
  {
    const double y = b0_ * x + b1_ * joint.x1 + b2_ * joint.x2 - a1_ * joint.y1 - a2_ * joint.y2;
    joint.x2 = joint.x1;
    joint.x1 = x;
    joint.y2 = joint.y1;
    joint.y1 = y;
    return y;
  }

  // The KRC's IPOC counts milliseconds.
  static constexpr double kSecondsPerIpoc = 0.001;
  bool filtered_ = false;
  double b0_ = 0.0, b1_ = 0.0, b2_ = 0.0, a1_ = 0.0, a2_ = 0.0;
  std::vector<Joint> joints_;
  uint64_t last_ipoc_ = 0;
  bool primed_ = false;
};

}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__VELOCITY_ESTIMATOR_HPP_
