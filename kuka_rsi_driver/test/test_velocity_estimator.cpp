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

#include <gtest/gtest.h>

#include <vector>

#include "kuka_rsi_driver/velocity_estimator.hpp"

using kuka_rsi_driver::VelocityEstimator;

namespace
{

constexpr double kPeriod = 0.004;
// The KRC's IPOC counts milliseconds, so one 4 ms cycle is 4 counts.
constexpr uint64_t kCycle = 4;

class VelocityEstimatorTest : public ::testing::Test
{
protected:
  void Configure(double cutoff_hz) { estimator_.Configure(1, cutoff_hz, kPeriod); }

  double Update(double position, uint64_t ipoc, bool * fresh = nullptr)
  {
    const bool updated = estimator_.Update({position}, ipoc, velocity_);
    if (fresh != nullptr)
    {
      *fresh = updated;
    }
    return velocity_[0];
  }

  VelocityEstimator estimator_;
  std::vector<double> velocity_{0.0};
};

}  // namespace

TEST_F(VelocityEstimatorTest, FirstSampleReportsStandstill)
{
  Configure(0.0);
  EXPECT_DOUBLE_EQ(Update(1.0, 100), 0.0);
}

TEST_F(VelocityEstimatorTest, UsesTheIpocDeltaForDt)
{
  Configure(0.0);
  Update(0.0, 100);
  // A dropped packet: two cycles between the samples
  EXPECT_DOUBLE_EQ(Update(0.008, 100 + 2 * kCycle), 1.0);
}

TEST_F(VelocityEstimatorTest, HoldsVelocityOnRepeatedIpoc)
{
  Configure(0.0);
  Update(0.0, 100);
  EXPECT_DOUBLE_EQ(Update(0.004, 100 + kCycle), 1.0);

  bool fresh = true;
  EXPECT_DOUBLE_EQ(Update(0.004, 100 + kCycle, &fresh), 1.0);
  EXPECT_FALSE(fresh);

  // The next sample spans both cycles, so no double step either
  EXPECT_DOUBLE_EQ(Update(0.012, 100 + 3 * kCycle), 1.0);
}

TEST_F(VelocityEstimatorTest, ResetStartsFromStandstill)
{
  Configure(0.0);
  Update(0.0, 100);
  Update(0.004, 100 + kCycle);
  estimator_.Reset();
  EXPECT_DOUBLE_EQ(Update(5.0, 900), 0.0);
}

TEST_F(VelocityEstimatorTest, LowPassSettlesOnTheInputAndSmoothsAStep)
{
  Configure(50.0);
  uint64_t ipoc = 100;
  double position = 0.0;
  Update(position, ipoc);
  position += 0.004;
  ipoc += kCycle;
  const double first = Update(position, ipoc);
  EXPECT_GT(first, 0.0);
  EXPECT_LT(first, 0.5) << "a 1 rad/s step passed unfiltered";

  for (int i = 0; i < 100; i++)
  {
    position += 0.004;
    ipoc += kCycle;
    Update(position, ipoc);
  }
  EXPECT_NEAR(velocity_[0], 1.0, 1e-6);
}
