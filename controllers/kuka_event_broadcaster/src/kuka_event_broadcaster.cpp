// Copyright 2024 KUKA Hungaria Kft.
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

#include "kuka_event_broadcaster/kuka_event_broadcaster.hpp"

#include <exception>

#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"

namespace kuka_controllers
{
controller_interface::CallbackReturn EventBroadcaster::on_init()
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
  auto event_publisher = get_node()->create_publisher<kuka_driver_interfaces::msg::HardwareEvent>(
    "~/hardware_event", rclcpp::SystemDefaultsQoS());
  event_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<kuka_driver_interfaces::msg::HardwareEvent>>(
      event_publisher);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration EventBroadcaster::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

std::string EventBroadcaster::ComposeInterfaceName(
  const std::string & robot_prefix, const std::string & interface_group,
  const std::string & interface_name)
{
  if (robot_prefix.empty())
  {
    return interface_group + "/" + interface_name;
  }
  else
  {
    return robot_prefix + "_" + interface_group + "/" + interface_name;
  }
}

controller_interface::InterfaceConfiguration EventBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // An empty robot_prefixes array is parsed as NOT_SET in this startup path and throws an
  // exception,
  //  so it is not necessary to check for it here. Default is [""] for single-robot/no-prefix
  //  behavior.
  for (const auto & robot_prefix : params_.robot_prefixes)
  {
    config.names.emplace_back(ComposeInterfaceName(
      robot_prefix, hardware_interface::STATE_PREFIX, hardware_interface::SERVER_STATE));
  }

  return config;
}

controller_interface::CallbackReturn EventBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  if (params_.robot_prefixes.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'robot_prefixes' must not be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  event_robot_prefixes_ = params_.robot_prefixes;

  last_events_.assign(event_robot_prefixes_.size(), 0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EventBroadcaster::on_activate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn EventBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type EventBroadcaster::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Publish every server_state change, one message per robot
  for (size_t i = 0; i < state_interfaces_.size(); ++i)
  {
    const auto current_event =
      static_cast<uint8_t>(state_interfaces_[i].get_optional().value_or(last_events_[i]));

    if (current_event != last_events_[i])
    {
      last_events_[i] = current_event;
      event_msg_.robot_name = event_robot_prefixes_[i];
      event_msg_.event = current_event;
      event_publisher_->try_publish(event_msg_);
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace kuka_controllers

PLUGINLIB_EXPORT_CLASS(
  kuka_controllers::EventBroadcaster, controller_interface::ControllerInterface)
