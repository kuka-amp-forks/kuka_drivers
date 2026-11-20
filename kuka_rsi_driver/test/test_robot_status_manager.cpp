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

#include <map>
#include <string>

#include "kuka_rsi_driver/robot_status_manager.hpp"

using kuka::external::control::ControlMode;
using kuka::external::control::OperationMode;
using kuka::external::control::kss::CycleTime;
using kuka::external::control::kss::StatusUpdate;
using kuka_rsi_driver::StatusManager;

namespace
{

// Every field at a value distinguishable from StatusUpdate::Reset()'s defaults, so a field
// Pack()/Unpack() or StatusInterfaces::operator= drops reads back as the default and is caught.
StatusUpdate AllFieldsSet()
{
  StatusUpdate update;
  update.control_mode_ = ControlMode::JOINT_IMPEDANCE_CONTROL;
  update.cycle_time_ = CycleTime::RSI_4MS;
  update.drives_powered_ = true;
  update.emergency_stop_ = true;
  update.guard_stop_ = true;
  update.in_motion_ = true;
  update.motion_possible_ = true;
  update.operation_mode_ = OperationMode::EXT;
  update.robot_stopped_ = true;
  update.brake_test_requested_ = true;
  update.brake_test_warning_ = true;
  update.stop_message_active_ = true;
  update.move_enable_sent_ = true;
  return update;
}

}  // namespace

// Pack() and Unpack() each hand-list the StatusUpdate fields; a field dropped from one would
// silently read back as its Reset() default instead of round-tripping.
TEST(RobotStatusManager, PackUnpackRoundTripsEveryField)
{
  StatusManager manager;
  const StatusUpdate sent = AllFieldsSet();
  manager.SetStatusInterfaces(sent);
  manager.UpdateStateInterfaces();

  std::map<std::string, double> values;
  for (const auto & [name, value] : manager.StateInterfaceValues())
  {
    values[name] = *value;
  }

  EXPECT_EQ(values.at(hardware_interface::CONTROL_MODE), static_cast<double>(sent.control_mode_));
  EXPECT_EQ(values.at(hardware_interface::CYCLE_TIME), static_cast<double>(sent.cycle_time_));
  EXPECT_EQ(values.at(hardware_interface::DRIVES_POWERED), 1.0);
  EXPECT_EQ(values.at(hardware_interface::EMERGENCY_STOP), 1.0);
  EXPECT_EQ(values.at(hardware_interface::GUARD_STOP), 1.0);
  EXPECT_EQ(values.at(hardware_interface::IN_MOTION), 1.0);
  EXPECT_EQ(values.at(hardware_interface::MOTION_POSSIBLE), 1.0);
  EXPECT_EQ(
    values.at(hardware_interface::OPERATION_MODE), static_cast<double>(sent.operation_mode_));
  EXPECT_EQ(values.at(hardware_interface::ROBOT_STOPPED), 1.0);
  EXPECT_EQ(values.at(hardware_interface::BRAKE_TEST_DUE), 1.0);
  EXPECT_EQ(values.at(hardware_interface::BRAKE_TEST_WARNING), 1.0);

  EXPECT_TRUE(manager.IsKrcInExtMode());
  EXPECT_TRUE(manager.DrivesPowered());
  EXPECT_TRUE(manager.IsEmergencyStopActive());
  EXPECT_TRUE(manager.IsMotionPossible());
  EXPECT_TRUE(manager.IsGuardStopActive());
}

// Invalidate() resets every field; a field Reset() misses would keep a stale non-default value.
TEST(RobotStatusManager, InvalidateClearsEveryField)
{
  StatusManager manager;
  manager.SetStatusInterfaces(AllFieldsSet());
  manager.UpdateStateInterfaces();
  manager.Invalidate();
  manager.UpdateStateInterfaces();

  for (const auto & [name, value] : manager.StateInterfaceValues())
  {
    EXPECT_EQ(*value, 0.0) << name;
  }
  EXPECT_TRUE(manager.StatusPending());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
