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

#ifndef KUKA_RSI_DRIVER__HARDWARE_INTERFACE_MXA_RSI_HPP_
#define KUKA_RSI_DRIVER__HARDWARE_INTERFACE_MXA_RSI_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface_rsi_base.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "kuka/external-control-sdk/kss/mxa/robot_interface.h"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_rsi_driver/visibility_control.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using InitializationData = kuka::external::control::kss::InitializationData;
using RsiCycleTime = kuka::external::control::kss::CycleTime;

namespace kuka_rsi_driver
{
class KukaMxaRsiHardwareInterface : public kuka_rsi_driver::KukaRSIHardwareInterfaceBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaMxaRsiHardwareInterface)

  KUKA_RSI_DRIVER_PUBLIC KukaMxaRsiHardwareInterface()
  : KukaRSIHardwareInterfaceBase("KukaMxaRsiHardwareInterface")
  {
  }

  KUKA_RSI_DRIVER_PUBLIC
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  KUKA_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  KUKA_RSI_DRIVER_PUBLIC CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  KUKA_RSI_DRIVER_PUBLIC
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  KUKA_RSI_DRIVER_PUBLIC void mxa_init(const InitializationData &);

  // Latches e-stop/guard stop from the mxA thread itself, in any lifecycle state.
  KUKA_RSI_DRIVER_PUBLIC void OnStatusUpdate(
    const kuka::external::control::kss::StatusUpdate & update) override;

protected:
  KUKA_RSI_DRIVER_LOCAL void CreateRobotInstance(
    const kuka::external::control::kss::Configuration &) override;

private:
  // Latches the first mxA status that forbids motion; true when this call latched it.
  KUKA_RSI_DRIVER_LOCAL bool LatchStatusStop();

  // The newest mxA telegram's age once past the bound with none still queued, otherwise zero.
  KUKA_RSI_DRIVER_LOCAL std::chrono::milliseconds MxaSilence() const;

  // Bound on the age of the newest mxA telegram, below the SDK's own 100 ms receive timeout.
  static constexpr std::chrono::milliseconds DEFAULT_MXA_STATUS_TIMEOUT{60};
  // Mirrors kuka::external::control::kss::mxa::Client::kUDPTimeoutMs (client.h); keep in sync.
  static constexpr std::chrono::milliseconds SDK_MXA_RECEIVE_TIMEOUT{100};
  std::chrono::milliseconds mxa_status_timeout_{DEFAULT_MXA_STATUS_TIMEOUT};
  // Set once read() has exported an mxA loss as server_state; the next write() or read() errors.
  bool mxa_loss_exported_ = false;
  // E-stop and guard stop of the last telegram, mxA thread only. Telegrams come only on a change,
  // so a flag still set is delivered again with every unrelated one; kept across a reconfigure
  // for the same reason.
  bool last_emergency_stop_ = false;
  bool last_guard_stop_ = false;

  InitSequenceReport init_report_;
  std::mutex init_mtx_;
  std::condition_variable init_cv_;

  bool verify_robot_model_;
  std::atomic<bool> stop_requested_{false};
};
}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__HARDWARE_INTERFACE_MXA_RSI_HPP_
