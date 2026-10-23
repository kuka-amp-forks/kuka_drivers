// Copyright 2022 KUKA Hungaria Kft.
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

#include "kuka_control_mode_handler/kuka_control_mode_handler.hpp"

#include <exception>

namespace kuka_controllers
{
controller_interface::CallbackReturn ControlModeHandler::on_init()
{
  try
  {
    auto param_listener = std::make_shared<ParamListener>(get_node());
    params_ = param_listener->get_params();
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize parameters: %s", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ControlModeHandler::command_interface_configuration()
  const
{
  // The RSI hardware components run joint position control only and export no control_mode
  // command interface.
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration ControlModeHandler::state_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn ControlModeHandler::on_configure(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ControlModeHandler::on_activate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ControlModeHandler::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ControlModeHandler::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::ControlModeHandler, controller_interface::ControllerInterface)
