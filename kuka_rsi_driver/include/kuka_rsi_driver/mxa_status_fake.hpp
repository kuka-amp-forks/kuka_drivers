// Copyright 2026 KUKA
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

#ifndef KUKA_RSI_DRIVER__MXA_STATUS_FAKE_HPP_
#define KUKA_RSI_DRIVER__MXA_STATUS_FAKE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "kuka/external-control-sdk/kss/configuration.h"
#include "kuka/external-control-sdk/kss/initialization_data.h"
#include "kuka/external-control-sdk/kss/rsi/robot_interface.h"
#include "kuka/external-control-sdk/kss/status_update.h"

namespace kuka_rsi_driver
{

// Test stand-in for kss::mxa::Robot. Keeps the plain RSI stream and replaces the mxA link
// with a loopback UDP socket that tests write status updates to, one text datagram each:
//   "<field>=<int> ..."  fields of kss::StatusUpdate, e.g. "emergency_stop=1 operation_mode=4"
//   "period_ms=<int>"    telegram cadence; 0 stops the cadence
//   "stall" / "resume"   stop or restart delivery to the driver
//   "error=<text>" / "stopped=<text>"  raise OnError / OnStopped on the event handler
// Only reachable through the mxa_status_fake_port hardware parameter.
class MxaStatusFake : public kuka::external::control::kss::rsi::Robot
{
public:
  using Status = kuka::external::control::Status;
  using StatusUpdate = kuka::external::control::kss::StatusUpdate;

  MxaStatusFake(
    const kuka::external::control::kss::Configuration & config, uint16_t port,
    rclcpp::Logger logger);
  ~MxaStatusFake() override;

  Status Setup() override;
  Status StartControlling(kuka::external::control::ControlMode) override;
  Status StopControlling() override;
  Status CancelRsiProgram() override;
  Status TurnOnDrives() override;
  Status TurnOffDrives() override;
  Status SetCycleTime(kuka::external::control::kss::CycleTime) override;
  Status RegisterEventHandler(
    std::unique_ptr<kuka::external::control::EventHandler> && handler) override;
  Status RegisterEventHandlerExtension(
    std::unique_ptr<kuka::external::control::kss::IEventHandlerExtension> && extension) override;
  Status RegisterStatusResponseHandler(
    std::unique_ptr<kuka::external::control::kss::IStatusUpdateHandler> && handler) override;

private:
  void Serve();
  void Apply(const std::string & datagram);
  bool ApplyToken(const std::string & token);
  void SetDrivesPowered(bool powered);
  void Dispatch();

  const uint16_t port_;
  const rclcpp::Logger logger_;
  int fd_ = -1;
  std::thread thread_;
  std::atomic<bool> running_{false};
  bool rsi_running_ = false;

  std::mutex mtx_;
  StatusUpdate status_;
  StatusUpdate delivered_;
  bool stalled_ = false;
  std::chrono::milliseconds period_{0};
  uint64_t telegrams_ = 0;
  std::unique_ptr<kuka::external::control::EventHandler> event_handler_;
  std::unique_ptr<kuka::external::control::kss::IStatusUpdateHandler> status_handler_;
};

}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__MXA_STATUS_FAKE_HPP_
