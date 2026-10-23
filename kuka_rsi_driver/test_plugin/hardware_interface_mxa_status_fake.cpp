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

#include <memory>
#include <string>

#include "pluginlib/class_list_macros.hpp"

#include "hardware_interface_mxa_status_fake.hpp"
#include "mxa_status_fake.hpp"

namespace kuka_rsi_driver
{

CallbackReturn KukaMxaRsiStatusFakeHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (KukaMxaRsiHardwareInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  const auto it = info_.hardware_parameters.find("mxa_status_fake_port");
  if (it == info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(logger_, "mxa_status_fake_port is required");
    return CallbackReturn::ERROR;
  }
  try
  {
    std::size_t parsed = 0;
    const int64_t port = std::stoll(it->second, &parsed);
    if (parsed == it->second.size() && port > 0 && port <= UINT16_MAX)
    {
      fake_port_ = static_cast<uint16_t>(port);
      return CallbackReturn::SUCCESS;
    }
  }
  catch (const std::exception &)
  {
  }
  RCLCPP_ERROR(
    logger_, "mxa_status_fake_port must be a UDP port number, got '%s'", it->second.c_str());
  return CallbackReturn::ERROR;
}

void KukaMxaRsiStatusFakeHardwareInterface::CreateRobotInstance(
  const kuka::external::control::kss::Configuration & config)
{
  RCLCPP_WARN(logger_, "mxA status fake on port %u, no mxA connection", fake_port_);
  robot_ptr_ = std::make_unique<MxaStatusFake>(config, fake_port_, logger_);
}

}  // namespace kuka_rsi_driver

PLUGINLIB_EXPORT_CLASS(
  kuka_rsi_driver::KukaMxaRsiStatusFakeHardwareInterface, hardware_interface::SystemInterface)
