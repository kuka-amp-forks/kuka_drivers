// Copyright 2025 KUKA Hungaria Kft.
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

#ifndef KUKA_RSI_DRIVER__ROBOT_STATUS_MANAGER_HPP_
#define KUKA_RSI_DRIVER__ROBOT_STATUS_MANAGER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "kuka/external-control-sdk/kss/status_update.h"
#include "kuka_drivers_core/hardware_interface_types.hpp"

namespace kuka_rsi_driver
{

class StatusInterfaces
{
public:
  StatusInterfaces & operator=(const kuka::external::control::kss::StatusUpdate & update)
  {
    control_mode_ = static_cast<double>(update.control_mode_);
    cycle_time_ = static_cast<double>(update.cycle_time_);
    drives_powered_ = static_cast<double>(update.drives_powered_);
    emergency_stop_ = static_cast<double>(update.emergency_stop_);
    guard_stop_ = static_cast<double>(update.guard_stop_);
    in_motion_ = static_cast<double>(update.in_motion_);
    motion_possible_ = static_cast<double>(update.motion_possible_);
    operation_mode_ = static_cast<double>(update.operation_mode_);
    robot_stopped_ = static_cast<double>(update.robot_stopped_);
    brake_test_due_ = static_cast<double>(update.brake_test_requested_);
    brake_test_warning_ = static_cast<double>(update.brake_test_warning_);
    return *this;
  }

  // The value behind each exported status state interface, by interface name.
  std::vector<std::pair<std::string, const double *>> StateInterfaceValues() const
  {
    return {
      {hardware_interface::CONTROL_MODE, &control_mode_},
      {hardware_interface::CYCLE_TIME, &cycle_time_},
      {hardware_interface::DRIVES_POWERED, &drives_powered_},
      {hardware_interface::EMERGENCY_STOP, &emergency_stop_},
      {hardware_interface::GUARD_STOP, &guard_stop_},
      {hardware_interface::IN_MOTION, &in_motion_},
      {hardware_interface::MOTION_POSSIBLE, &motion_possible_},
      {hardware_interface::OPERATION_MODE, &operation_mode_},
      {hardware_interface::ROBOT_STOPPED, &robot_stopped_},
      {hardware_interface::BRAKE_TEST_DUE, &brake_test_due_},
      {hardware_interface::BRAKE_TEST_WARNING, &brake_test_warning_}};
  }

  bool IsOperationModeExt() const
  {
    return static_cast<uint8_t>(operation_mode_) ==
           static_cast<uint8_t>(kuka::external::control::OperationMode::EXT);
  }

  bool DrivesPowered() const { return static_cast<bool>(drives_powered_); }

  bool IsEmergencyStopActive() const { return static_cast<bool>(emergency_stop_); }

  bool IsMotionPossible() const { return static_cast<bool>(motion_possible_); }

  bool IsGuardStopActive() const { return static_cast<bool>(guard_stop_); }

  bool IsBrakeTestDue() const { return static_cast<bool>(brake_test_due_); }

  std::string Describe() const
  {
    return "emergency_stop=" + std::to_string(static_cast<int>(emergency_stop_)) +
           " guard_stop=" + std::to_string(static_cast<int>(guard_stop_)) +
           " drives_powered=" + std::to_string(static_cast<int>(drives_powered_)) +
           " motion_possible=" + std::to_string(static_cast<int>(motion_possible_)) +
           " operation_mode=" + std::to_string(static_cast<int>(operation_mode_));
  }

private:
  double control_mode_ = 0.0;
  double cycle_time_ = 0.0;
  double drives_powered_ = 0.0;
  double emergency_stop_ = 0.0;
  double guard_stop_ = 0.0;
  double in_motion_ = 0.0;
  double motion_possible_ = 0.0;
  double operation_mode_ = 0.0;
  double robot_stopped_ = 0.0;
  double brake_test_due_ = 0.0;
  double brake_test_warning_ = 0.0;
};

class StatusManager
{
public:
  std::vector<std::pair<std::string, const double *>> StateInterfaceValues() const
  {
    return status_interfaces_.StateInterfaceValues();
  }

  // Called on the mxA (or EKI) thread; read() takes the update without a lock.
  void SetStatusInterfaces(const kuka::external::control::kss::StatusUpdate & update)
  {
    actual_status_.store(Pack(update), std::memory_order_release);
  }

  // Drops the status of an earlier link; StatusPending() holds until the new link reports one.
  void Invalidate()
  {
    kuka::external::control::kss::StatusUpdate reset;
    reset.Reset();
    actual_status_.store(Pack(reset) | PENDING_BIT, std::memory_order_release);
    status_interfaces_ = reset;
  }

  bool StatusPending() const
  {
    return (actual_status_.load(std::memory_order_acquire) & PENDING_BIT) != 0;
  }

  // The flag accessors below read this copy, which only the thread calling read() and the
  // lifecycle transitions touch; the hardware component serializes those.
  void UpdateStateInterfaces()
  {
    status_interfaces_ = Unpack(actual_status_.load(std::memory_order_acquire));
  }

  bool IsKrcInExtMode() const { return status_interfaces_.IsOperationModeExt(); }

  bool DrivesPowered() const { return status_interfaces_.DrivesPowered(); }

  bool IsEmergencyStopActive() const { return status_interfaces_.IsEmergencyStopActive(); }

  bool IsMotionPossible() const { return status_interfaces_.IsMotionPossible(); }

  bool IsGuardStopActive() const { return status_interfaces_.IsGuardStopActive(); }

  std::string Describe() const { return status_interfaces_.Describe(); }

  const StatusInterfaces & Current() const { return status_interfaces_; }

  // Called on the mxA thread for every received telegram, changed flags or not.
  void StampTelegram()
  {
    telegrams_.fetch_add(1, std::memory_order_relaxed);
    last_telegram_ns_.store(
      std::chrono::steady_clock::now().time_since_epoch().count(), std::memory_order_relaxed);
  }

  uint64_t TelegramCount() const { return telegrams_.load(std::memory_order_relaxed); }

  std::chrono::nanoseconds TelegramAge() const
  {
    return std::chrono::steady_clock::now().time_since_epoch() -
           std::chrono::steady_clock::duration(last_telegram_ns_.load(std::memory_order_relaxed));
  }

private:
  using StatusUpdate = kuka::external::control::kss::StatusUpdate;

  // Pack()/Unpack() and StatusInterfaces::operator= each hand-list StatusUpdate's fields; this
  // catches the SDK adding or removing one before it is silently dropped from all three.
  static_assert(
    sizeof(StatusUpdate) == 13,
    "StatusUpdate changed size: update the field lists in StatusInterfaces::operator=, Pack() "
    "and Unpack(), and this assert, in robot_status_manager.hpp");

  static constexpr uint64_t PENDING_BIT = uint64_t{1} << 63;

  static uint64_t Pack(const StatusUpdate & update)
  {
    uint64_t bits = static_cast<uint64_t>(update.control_mode_) |
                    static_cast<uint64_t>(update.cycle_time_) << 8 |
                    static_cast<uint64_t>(update.operation_mode_) << 16;
    const bool flags[] = {update.drives_powered_,       update.emergency_stop_,
                          update.guard_stop_,           update.in_motion_,
                          update.motion_possible_,      update.robot_stopped_,
                          update.brake_test_requested_, update.brake_test_warning_,
                          update.stop_message_active_,  update.move_enable_sent_};
    for (size_t i = 0; i < sizeof(flags) / sizeof(flags[0]); ++i)
    {
      bits |= static_cast<uint64_t>(flags[i]) << (24 + i);
    }
    return bits;
  }

  static StatusUpdate Unpack(uint64_t bits)
  {
    StatusUpdate update;
    update.control_mode_ = static_cast<kuka::external::control::ControlMode>(bits & 0xFF);
    update.cycle_time_ = static_cast<kuka::external::control::kss::CycleTime>((bits >> 8) & 0xFF);
    update.operation_mode_ =
      static_cast<kuka::external::control::OperationMode>((bits >> 16) & 0xFF);
    bool * flags[] = {&update.drives_powered_,       &update.emergency_stop_,
                      &update.guard_stop_,           &update.in_motion_,
                      &update.motion_possible_,      &update.robot_stopped_,
                      &update.brake_test_requested_, &update.brake_test_warning_,
                      &update.stop_message_active_,  &update.move_enable_sent_};
    for (size_t i = 0; i < sizeof(flags) / sizeof(flags[0]); ++i)
    {
      *flags[i] = ((bits >> (24 + i)) & 1) != 0;
    }
    return update;
  }

  StatusInterfaces status_interfaces_;  // Used as ROS 2 state interface
  std::atomic<uint64_t> actual_status_{0};
  std::atomic<std::chrono::steady_clock::rep> last_telegram_ns_{0};
  std::atomic<uint64_t> telegrams_{0};
};

}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__ROBOT_STATUS_MANAGER_HPP_
