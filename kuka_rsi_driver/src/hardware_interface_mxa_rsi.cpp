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

#include <chrono>
#include <regex>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_rsi_driver/event_observers.hpp"
#include "kuka_rsi_driver/hardware_interface_mxa_rsi.hpp"

namespace kuka_rsi_driver
{

CallbackReturn KukaMxaRsiHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (KukaRSIHardwareInterfaceBase::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Controller IP: %s", info_.hardware_parameters.at("controller_ip").c_str());

  auto it = info_.hardware_parameters.find("verify_robot_model");
  verify_robot_model_ = (it != info_.hardware_parameters.end() && it->second == "True");
  RCLCPP_INFO(
    logger_, "Robot model verification: %s", verify_robot_model_ ? "enabled" : "disabled");

  if (!ReadTimeoutParameter(
        "mxa_status_timeout_ms", DEFAULT_MXA_STATUS_TIMEOUT, mxa_status_timeout_))
  {
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    logger_, "mxA status timeout: %ld ms", static_cast<int64_t>(mxa_status_timeout_.count()));

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KukaMxaRsiHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces = KukaRSIHardwareInterfaceBase::export_state_interfaces();

  control_state_.status_manager.RegisterStateInterfaces(state_interfaces, interface_prefix_);

  return state_interfaces;
}

CallbackReturn KukaMxaRsiHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (mxa_status_timeout_ >= SDK_MXA_RECEIVE_TIMEOUT)
  {
    RCLCPP_ERROR(
      logger_,
      "mxa_status_timeout_ms is %ld, it must stay below the SDK's %ld ms mxA receive timeout",
      static_cast<int64_t>(mxa_status_timeout_.count()),
      static_cast<int64_t>(SDK_MXA_RECEIVE_TIMEOUT.count()));
    return CallbackReturn::FAILURE;
  }

  kuka::external::control::kss::Configuration mxa_config;
  mxa_config.installed_interface =
    kuka::external::control::kss::Configuration::InstalledInterface::MXA_RSI;
  mxa_config.kli_ip_address = info_.hardware_parameters.at("controller_ip");
  mxa_config.client_ip = info_.hardware_parameters.at("client_ip");
  mxa_config.client_port = std::stoi(info_.hardware_parameters.at("client_port"));
  mxa_config.mxa_client_port = std::stoi(info_.hardware_parameters.at("mxa_client_port"));

  {
    std::lock_guard<std::mutex> lk{init_mtx_};
    init_report_ = {};
  }

  if (!SetupRobot(
        mxa_config, std::make_unique<EventObserver>(this),
        std::make_unique<MxaEventHandlerExtension>(this)))
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger_, "Network setup successful - Controller: %s, mxA port: %d, RSI listening on %s:%d",
    mxa_config.kli_ip_address.c_str(), mxa_config.mxa_client_port, mxa_config.client_ip.c_str(),
    mxa_config.client_port);

  auto status = robot_ptr_->RegisterStatusResponseHandler(
    std::make_unique<StatusUpdateHandler>(&control_state_.status_manager));
  if (status.return_code != kuka::external::control::ReturnCode::OK)
  {
    RCLCPP_ERROR(logger_, "Registering status response handler failed: %s", status.message);
    return CallbackReturn::ERROR;
  }

  // Wait for the response to arrive from the controller
  std::unique_lock<std::mutex> lk{init_mtx_};
  if (!init_cv_.wait_for(lk, configure_timeout_, [this] { return init_report_.sequence_complete; }))
  {
    lk.unlock();
    RCLCPP_ERROR_STREAM(
      logger_, "No initialization report from the mxA server at "
                 << mxa_config.kli_ip_address << " within " << configure_timeout_.count() << " ms");
    robot_ptr_.reset();
    return CallbackReturn::FAILURE;
  }

  if (verify_robot_model_)
  {
    if (!init_report_.ok)
    {
      RCLCPP_ERROR(
        logger_, "The driver is incompatible with the current hardware and software setup: %s",
        init_report_.reason.c_str());
      robot_ptr_.reset();
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger_, "The driver is compatible with the current hardware and software setup");
  }
  else
  {
    RCLCPP_INFO(logger_, "Robot model verification is disabled, proceeding with the connection.");
  }

  control_state_.status_manager.StampTelegram();
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaMxaRsiHardwareInterface::on_activate(const rclcpp_lifecycle::State & state)
{
  return KukaRSIHardwareInterfaceBase::extended_activation(state);
}

CallbackReturn KukaMxaRsiHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & state)
{
  return KukaRSIHardwareInterfaceBase::extended_deactivation(state);
}

// A silent mxA link is a connection loss in every state; ERROR releases the client.
CallbackReturn KukaMxaRsiHardwareInterface::on_error(const rclcpp_lifecycle::State & state)
{
  if (LatchStopReason(kuka_drivers_core::StopReason::MXA_CONNECTION_LOST))
  {
    RCLCPP_ERROR(logger_, "Stopping: hardware error, treated as an mxA connection loss");
  }
  RCLCPP_ERROR(
    logger_, "Releasing the mxA client, stop_reason reads '%s'; configure again to reconnect",
    kuka_drivers_core::StopReasonName(event_state_.stop_reason.load()));
  return KukaRSIHardwareInterfaceBase::on_error(state);
}

return_type KukaMxaRsiHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & duration)
{
  const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
    control_state_.status_manager.TelegramAge());
  if (age > mxa_status_timeout_)
  {
    if (LatchStopReason(kuka_drivers_core::StopReason::MXA_CONNECTION_LOST))
    {
      RCLCPP_ERROR(
        logger_, "Stopping: no mxA telegram within %ld ms, the newest is %ld ms old",
        static_cast<int64_t>(mxa_status_timeout_.count()), static_cast<int64_t>(age.count()));
    }
    UpdateStopReasonState();
    return return_type::ERROR;
  }

  control_state_.status_manager.UpdateStateInterfaces();
  if (runtime_state_.is_active)
  {
    LatchStatusStop();
  }

  return KukaRSIHardwareInterfaceBase::read(time, duration);
}

bool KukaMxaRsiHardwareInterface::LatchStatusStop()
{
  using kuka_drivers_core::StopReason;
  auto & status = control_state_.status_manager;

  StopReason reason = StopReason::NONE;
  if (status.IsEmergencyStopActive())
  {
    reason = StopReason::EMERGENCY_STOP;
  }
  else if (status.IsGuardStopActive())
  {
    reason = StopReason::GUARD_STOP;
  }
  else if (!status.DrivesPowered())
  {
    reason = StopReason::DRIVES_NOT_POWERED;
  }
  else if (!status.IsMotionPossible())
  {
    reason = StopReason::MOTION_NOT_POSSIBLE;
  }
  else if (!status.IsKrcInExtMode())
  {
    reason = StopReason::NOT_IN_EXT;
  }

  if (reason == StopReason::NONE || !LatchStopReason(reason))
  {
    return false;
  }
  set_server_event(kuka_drivers_core::HardwareEvent::ERROR);
  RCLCPP_ERROR(
    logger_, "Stopping: %s (%s, IPOC %lu)", kuka_drivers_core::StopReasonName(reason),
    status.Describe().c_str(), diagnostics_state_.last_ipoc);
  return true;
}

void KukaMxaRsiHardwareInterface::mxa_init(const InitializationData & init_data)
{
  if (init_data.GetTotalAxisCount() == 0)
  {
    RCLCPP_WARN(
      logger_,
      "Skipping robot model verification, as it is not supported for mxA versions below 4.0");
    init_report_ = {true, true, ""};
  }
  else
  {
    std::lock_guard<std::mutex> lk{init_mtx_};
    if (info_.joints.size() != init_data.GetTotalAxisCount())
    {
      std::ostringstream oss;
      oss << "Mismatch in axis count: Driver expects " << info_.joints.size()
          << ", but mxAutomation server reported " << init_data.GetTotalAxisCount();
      init_report_ = {true, false, oss.str()};
    }
    else
    {
      init_report_ = {true, true, ""};
    }
  }
  init_cv_.notify_one();
}

void KukaMxaRsiHardwareInterface::CreateRobotInstance(
  const kuka::external::control::kss::Configuration & config)
{
  robot_ptr_ = std::make_unique<kuka::external::control::kss::mxa::Robot>(config);
}

}  // namespace kuka_rsi_driver

PLUGINLIB_EXPORT_CLASS(
  kuka_rsi_driver::KukaMxaRsiHardwareInterface, hardware_interface::SystemInterface)
