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

#include "mxa_status_fake.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cinttypes>
#include <sstream>
#include <tuple>
#include <utility>

namespace kuka_rsi_driver
{

using kuka::external::control::ReturnCode;

MxaStatusFake::MxaStatusFake(
  const kuka::external::control::kss::Configuration & config, uint16_t port, rclcpp::Logger logger)
: kuka::external::control::kss::rsi::Robot(config), port_(port), logger_(std::move(logger))
{
  status_.Reset();
  delivered_.Reset();
}

MxaStatusFake::~MxaStatusFake()
{
  running_ = false;
  if (thread_.joinable())
  {
    thread_.join();
  }
  if (fd_ >= 0)
  {
    close(fd_);
  }
}

MxaStatusFake::Status MxaStatusFake::Setup()
{
  fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0)
  {
    return {ReturnCode::ERROR, "mxA status fake: socket() failed"};
  }
  const int reuse = 1;
  setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port_);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
  {
    return {ReturnCode::ERROR, "mxA status fake: bind() failed"};
  }

  running_ = true;
  thread_ = std::thread(&MxaStatusFake::Serve, this);
  RCLCPP_INFO(logger_, "mxA status fake listening on 127.0.0.1:%u", port_);

  return kuka::external::control::kss::rsi::Robot::Setup();
}

MxaStatusFake::Status MxaStatusFake::StartControlling(kuka::external::control::ControlMode mode)
{
  if (mode != kuka::external::control::ControlMode::JOINT_POSITION_CONTROL)
  {
    std::ostringstream msg;
    msg << "mxA status fake: refusing control mode " << static_cast<int>(mode)
        << ", only JOINT_POSITION_CONTROL is supported";
    return {ReturnCode::ERROR, msg.str().c_str()};
  }
  std::lock_guard<std::mutex> lk{mtx_};
  if (cancel_pending_ > 0)
  {
    return {ReturnCode::ERROR, "RSI program cancel still in progress"};
  }
  rsi_running_ = true;
  if (event_handler_)
  {
    event_handler_->OnSampling();
  }
  return {ReturnCode::OK, "mxA status fake: RSI start requested"};
}

MxaStatusFake::Status MxaStatusFake::StopControlling()
{
  if (!rsi_running_)
  {
    return {ReturnCode::WARN, "Control already stopped"};
  }
  auto result = kuka::external::control::kss::rsi::Robot::StopControlling();
  rsi_running_ = false;
  return result;
}

// Like a failed sendto, leaves the SDK's request pending so the stop packet still has a target.
MxaStatusFake::Status MxaStatusFake::SendControlSignal()
{
  if (send_fail_)
  {
    return {ReturnCode::ERROR, "mxA status fake: send failure injected"};
  }
  return kuka::external::control::kss::rsi::Robot::SendControlSignal();
}

// Blocking form, like the SDK: no bound, so it never returns while status is frozen.
MxaStatusFake::Status MxaStatusFake::CancelRsiProgram()
{
  RequestRsiProgramCancel();
  std::unique_lock<std::mutex> lk{mtx_};
  cancel_cv_.wait(lk, [this] { return cancelled_; });
  return {ReturnCode::OK, "RSI program cancelled"};
}

MxaStatusFake::Status MxaStatusFake::RequestRsiProgramCancel()
{
  std::lock_guard<std::mutex> lk{mtx_};
  rsi_running_ = false;
  cancelled_ = cancel_cycles_ <= 0;
  cancel_pending_ = cancelled_ ? 0 : cancel_cycles_;
  RCLCPP_INFO(
    logger_, "mxA status fake: RSI program cancel requested, done after %d cycles", cancel_cycles_);
  return {ReturnCode::OK, "RSI program cancel requested"};
}

bool MxaStatusFake::IsRsiProgramCancelled() const
{
  std::lock_guard<std::mutex> lk{mtx_};
  return cancelled_;
}

MxaStatusFake::Status MxaStatusFake::TurnOnDrives()
{
  SetDrivesPowered(true);
  return {ReturnCode::OK};
}

MxaStatusFake::Status MxaStatusFake::TurnOffDrives()
{
  SetDrivesPowered(false);
  return {ReturnCode::OK};
}

MxaStatusFake::Status MxaStatusFake::SetCycleTime(
  kuka::external::control::kss::CycleTime cycle_time)
{
  if (cycle_time != kuka::external::control::kss::CycleTime::RSI_4MS)
  {
    std::ostringstream msg;
    msg << "mxA status fake: refusing cycle time "
        << kuka::external::control::kss::CycleTimeToString(cycle_time)
        << ", only 4 ms is supported";
    return {ReturnCode::ERROR, msg.str().c_str()};
  }
  return {ReturnCode::OK, "Cycle time set"};
}

MxaStatusFake::Status MxaStatusFake::RegisterEventHandler(
  std::unique_ptr<kuka::external::control::EventHandler> && handler)
{
  std::lock_guard<std::mutex> lk{mtx_};
  event_handler_ = std::move(handler);
  return {ReturnCode::OK};
}

MxaStatusFake::Status MxaStatusFake::RegisterEventHandlerExtension(
  std::unique_ptr<kuka::external::control::kss::IEventHandlerExtension> && extension)
{
  extension->OnConnected(
    kuka::external::control::kss::InitializationData(static_cast<uint8_t>(config_.dof), 0));
  return {ReturnCode::OK};
}

MxaStatusFake::Status MxaStatusFake::RegisterStatusResponseHandler(
  std::unique_ptr<kuka::external::control::kss::IStatusUpdateHandler> && handler)
{
  std::lock_guard<std::mutex> lk{mtx_};
  status_handler_ = std::move(handler);
  return {ReturnCode::OK};
}

void MxaStatusFake::Serve()
{
  std::array<char, 512> buf{};
  while (running_)
  {
    pollfd pfd{fd_, POLLIN, 0};
    if (poll(&pfd, 1, 10) > 0)
    {
      const auto n = recv(fd_, buf.data(), buf.size() - 1, 0);
      if (n > 0)
      {
        Apply(std::string(buf.data(), static_cast<size_t>(n)));
      }
    }

    std::lock_guard<std::mutex> lk{mtx_};
    if (!link_down_ && status_handler_)
    {
      ++telegrams_;
      status_handler_->OnTelegramReceived();
    }
    // Like the SDK, the cancel only progresses on a live link.
    if (!status_frozen_ && !link_down_ && cancel_pending_ > 0 && --cancel_pending_ == 0)
    {
      cancelled_ = true;
      cancel_cv_.notify_all();
      RCLCPP_INFO(
        logger_, "mxA status fake: RSI program cancelled after %d cycles", cancel_cycles_);
    }
  }
}

void MxaStatusFake::Apply(const std::string & datagram)
{
  RCLCPP_INFO(logger_, "mxA status fake: %s", datagram.c_str());
  std::istringstream tokens{datagram};
  std::string token;
  while (tokens >> token)
  {
    if (!ApplyToken(token))
    {
      RCLCPP_ERROR(logger_, "mxA status fake: unknown token '%s'", token.c_str());
    }
  }
  Dispatch();
}

bool MxaStatusFake::ApplyToken(const std::string & token)
{
  std::lock_guard<std::mutex> lk{mtx_};
  if (token == "freeze_status")
  {
    status_frozen_ = true;
    RCLCPP_INFO(logger_, "mxA status fake: status frozen after %" PRIu64 " telegrams", telegrams_);
    return true;
  }
  if (token == "unfreeze_status")
  {
    status_frozen_ = false;
    RCLCPP_INFO(logger_, "mxA status fake: status unfrozen");
    return true;
  }
  if (token == "link_down")
  {
    link_down_ = true;
    RCLCPP_INFO(logger_, "mxA status fake: link down after %" PRIu64 " telegrams", telegrams_);
    return true;
  }
  if (token == "link_up")
  {
    link_down_ = false;
    RCLCPP_INFO(logger_, "mxA status fake: link up");
    return true;
  }
  if (token == "report")
  {
    RCLCPP_INFO(
      logger_, "mxA status fake: rsi_running=%d cancel_pending=%d cancelled=%d drives_powered=%d",
      rsi_running_ ? 1 : 0, cancel_pending_, cancelled_ ? 1 : 0, status_.drives_powered_ ? 1 : 0);
    return true;
  }

  const auto eq = token.find('=');
  if (eq == std::string::npos)
  {
    return false;
  }
  const auto key = token.substr(0, eq);
  const auto value = token.substr(eq + 1);

  if (key == "error" || key == "stopped")
  {
    if (event_handler_ && key == "error")
    {
      event_handler_->OnError(value);
    }
    else if (event_handler_)
    {
      event_handler_->OnStopped(value);
    }
    return true;
  }

  int number = 0;
  try
  {
    number = std::stoi(value);
  }
  catch (const std::exception &)
  {
    return false;
  }
  const bool flag = number != 0;

  if (key == "cancel_cycles")
  {
    cancel_cycles_ = number;
  }
  else if (key == "drives_stuck")
  {
    drives_stuck_ = flag;
  }
  else if (key == "send_fail")
  {
    send_fail_ = flag;
  }
  else if (key == "control_mode")
  {
    status_.control_mode_ = static_cast<kuka::external::control::ControlMode>(number);
  }
  else if (key == "cycle_time")
  {
    status_.cycle_time_ = static_cast<kuka::external::control::kss::CycleTime>(number);
  }
  else if (key == "drives_powered")
  {
    status_.drives_powered_ = flag;
  }
  else if (key == "emergency_stop")
  {
    status_.emergency_stop_ = flag;
  }
  else if (key == "guard_stop")
  {
    status_.guard_stop_ = flag;
  }
  else if (key == "in_motion")
  {
    status_.in_motion_ = flag;
  }
  else if (key == "motion_possible")
  {
    status_.motion_possible_ = flag;
  }
  else if (key == "operation_mode")
  {
    status_.operation_mode_ = static_cast<kuka::external::control::OperationMode>(number);
  }
  else if (key == "robot_stopped")
  {
    status_.robot_stopped_ = flag;
  }
  else
  {
    return false;
  }
  return true;
}

void MxaStatusFake::SetDrivesPowered(bool powered)
{
  {
    std::lock_guard<std::mutex> lk{mtx_};
    if (drives_stuck_)
    {
      RCLCPP_INFO(logger_, "mxA status fake: drives %s request ignored", powered ? "on" : "off");
      return;
    }
    status_.drives_powered_ = powered;
  }
  Dispatch();
}

// Like the SDK, deliver only when the flags changed.
void MxaStatusFake::Dispatch()
{
  std::lock_guard<std::mutex> lk{mtx_};
  if (status_frozen_ || link_down_ || !status_handler_)
  {
    return;
  }
  const bool changed =
    std::tie(
      status_.control_mode_, status_.cycle_time_, status_.drives_powered_, status_.emergency_stop_,
      status_.guard_stop_, status_.in_motion_, status_.motion_possible_, status_.operation_mode_,
      status_.robot_stopped_) !=
    std::tie(
      delivered_.control_mode_, delivered_.cycle_time_, delivered_.drives_powered_,
      delivered_.emergency_stop_, delivered_.guard_stop_, delivered_.in_motion_,
      delivered_.motion_possible_, delivered_.operation_mode_, delivered_.robot_stopped_);
  if (!changed)
  {
    return;
  }
  delivered_ = status_;
  ++telegrams_;
  status_handler_->OnTelegramReceived();
  status_handler_->OnStatusUpdateReceived(delivered_);
}

}  // namespace kuka_rsi_driver
