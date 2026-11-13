// Copyright 2023 KUKA Hungaria Kft.
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

#ifndef KUKA_RSI_DRIVER__HARDWARE_INTERFACE_RSI_BASE_HPP_
#define KUKA_RSI_DRIVER__HARDWARE_INTERFACE_RSI_BASE_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "kuka/external-control-sdk/kss/configuration.h"
#include "kuka/external-control-sdk/kss/rsi/robot_interface.h"
#include "kuka_drivers_core/control_mode.hpp"
#include "kuka_drivers_core/stop_reason.hpp"
#include "kuka_rsi_driver/robot_status_manager.hpp"
#include "kuka_rsi_driver/visibility_control.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using RsiCycleTime = kuka::external::control::kss::CycleTime;

namespace kuka_rsi_driver
{

class KukaRSIHardwareInterfaceBase : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaRSIHardwareInterfaceBase)

  explicit KUKA_RSI_DRIVER_PUBLIC KukaRSIHardwareInterfaceBase(const std::string & logger_name)
  : SystemInterface(), logger_(rclcpp::get_logger(logger_name))
  {
  }

  virtual KUKA_RSI_DRIVER_PUBLIC ~KukaRSIHardwareInterfaceBase() = default;

  KUKA_RSI_DRIVER_PUBLIC
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  KUKA_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
    override;

  KUKA_RSI_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
    override;

  KUKA_RSI_DRIVER_PUBLIC CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

  KUKA_RSI_DRIVER_PUBLIC
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  KUKA_RSI_DRIVER_PUBLIC
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  KUKA_RSI_DRIVER_PUBLIC void set_server_event(kuka_drivers_core::HardwareEvent);

  // Latches an SDK stop or error event while active, and the KRC's stop message (mxA error 801)
  // in any state; called on the SDK's event thread.
  KUKA_RSI_DRIVER_PUBLIC void OnControlEvent(
    kuka_drivers_core::StopReason reason, const std::string & detail);

  // Joint position is the only command mode.
  KUKA_RSI_DRIVER_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // No-op here; the mxA interface overrides it to latch e-stop/guard stop while inactive.
  virtual KUKA_RSI_DRIVER_PUBLIC void OnStatusUpdate(
    const kuka::external::control::kss::StatusUpdate &)
  {
  }

protected:
  // The inhibitor slots, in export order: inhibitor/<name>/{active,latched,operator_action}.
  enum Inhibitor : size_t
  {
    ESTOP,
    GUARD,
    MODE,
    DRIVE_FAULT,
    BRAKE_TEST,
    RSI,
    INHIBITOR_COUNT
  };
  static constexpr std::array<const char *, INHIBITOR_COUNT> INHIBITOR_NAMES = {
    "estop", "guard", "mode", "drive_fault", "brake_test", "rsi"};
  // mode never latches: the key switch is its own release.
  static constexpr std::array<bool, INHIBITOR_COUNT> INHIBITOR_LATCHES = {true, true, false,
                                                                          true, true, true};
  // Fixed per slot: 1 when the release needs an operator.
  static constexpr std::array<double, INHIBITOR_COUNT> INHIBITOR_OPERATOR_ACTION = {1.0, 1.0, 1.0,
                                                                                    1.0, 1.0, 0.0};

  // Latches reason unless one is set already (the first cause wins); true when this call set it.
  // write() returns DEACTIVATE while it is set.
  KUKA_RSI_DRIVER_LOCAL bool LatchStopReason(kuka_drivers_core::StopReason reason);

  // Exports the six inhibitor slots; the mxA and EKI interfaces call it, RSI-only has no source.
  KUKA_RSI_DRIVER_LOCAL void ExportInhibitorSlots(
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> & state_interfaces);

  // Refreshes active from the sources and folds in the latch requests. read() calls it every
  // cycle; a latch outlives its source and clears only in ResetStopReason().
  KUKA_RSI_DRIVER_LOCAL void UpdateInhibitors();

  KUKA_RSI_DRIVER_LOCAL void UpdateInhibitorActive();

  // At the instant the component leaves ACTIVE on a fault: every active slot but mode latches,
  // and a stop with no slot active raises inhibitor/rsi, since only a lost command link explains
  // it.
  KUKA_RSI_DRIVER_LOCAL void LatchInhibitorsAtStop();

  // Latches a slot from any thread; read() applies it.
  KUKA_RSI_DRIVER_LOCAL void RequestInhibitorLatch(Inhibitor slot);

  KUKA_RSI_DRIVER_LOCAL bool SetupRobot(
    kuka::external::control::kss::Configuration & config,
    std::unique_ptr<kuka::external::control::EventHandler> event_handler,
    std::unique_ptr<kuka::external::control::kss::IEventHandlerExtension> extension);

  // Exported: the test plugin's vtable needs it since the mxA class no longer overrides it.
  virtual KUKA_RSI_DRIVER_PUBLIC void Read(const int64_t request_timeout);

  // False when the send failed; the caller decides whether that stops the component.
  KUKA_RSI_DRIVER_LOCAL bool Write();

  KUKA_RSI_DRIVER_LOCAL void ResetDiagnostics();

  // Clears the stop cause, the drive and RSI faults and every latch; only a successful activation
  // may call it.
  KUKA_RSI_DRIVER_LOCAL void ResetStopReason();

  KUKA_RSI_DRIVER_LOCAL void UpdateDriveFaultState();

  // Exports a state interface that mirrors value.
  KUKA_RSI_DRIVER_LOCAL void ExportState(
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> & state_interfaces,
    const std::string & prefix, const std::string & name, const double * value);

  KUKA_RSI_DRIVER_LOCAL void ExportCommand(
    std::vector<hardware_interface::CommandInterface::SharedPtr> & command_interfaces,
    const std::string & prefix, const std::string & name, double * value);

  // Copies the states into the exported interfaces; without wait_for_lock a busy one is skipped.
  KUKA_RSI_DRIVER_LOCAL void PublishStates(bool wait_for_lock);

  KUKA_RSI_DRIVER_LOCAL void PublishCommands();

  // Takes the controllers' commands; a busy interface keeps the last one.
  KUKA_RSI_DRIVER_LOCAL void FetchCommands();

  // Requests the RSI program cancel and drives-off without waiting, and marks the stop pending.
  KUKA_RSI_DRIVER_LOCAL void RequestStop();

  // Polled from read() while a stop is pending.
  KUKA_RSI_DRIVER_LOCAL void CheckStopComplete();

  // Drops the RSI packets queued before the start request; false when one could not be answered
  // or they keep arriving.
  KUKA_RSI_DRIVER_LOCAL bool DiscardQueuedPackets();

  // Reads the fault_injection hardware parameter; false when it is not a boolean.
  KUKA_RSI_DRIVER_LOCAL bool ReadFaultInjectionParameter();

  // Clears the fault injection commands; called at activation.
  KUKA_RSI_DRIVER_LOCAL void ResetFaultInjection();

  // Acts on a new gpio/stop_request or gpio/rsi_mute_ms command; called from write().
  KUKA_RSI_DRIVER_LOCAL void ApplyFaultInjection();

  // True while replies are withheld; latches the stop once the window has passed.
  KUKA_RSI_DRIVER_LOCAL bool RsiMuted();

  KUKA_RSI_DRIVER_LOCAL bool CheckJointInterfaces(
    const hardware_interface::ComponentInfo & joint) const;

  KUKA_RSI_DRIVER_LOCAL bool CheckJointCommandInterfaces(
    const hardware_interface::ComponentInfo & joint) const;

  KUKA_RSI_DRIVER_LOCAL bool CheckJointStateInterfaces(
    const hardware_interface::ComponentInfo & joint) const;

  KUKA_RSI_DRIVER_LOCAL void CopyGPIOStatesToCommands();

  KUKA_RSI_DRIVER_LOCAL void DeriveVelocityStates();

  virtual KUKA_RSI_DRIVER_LOCAL void CreateRobotInstance(
    const kuka::external::control::kss::Configuration &) = 0;

  KUKA_RSI_DRIVER_LOCAL kuka::external::control::kss::GPIOConfiguration ParseGPIOConfig(
    const hardware_interface::InterfaceInfo & info);

  // Reads the *_timeout_ms hardware parameters; false when one is set but not a positive integer.
  KUKA_RSI_DRIVER_LOCAL bool ReadTimeoutParameters();

  KUKA_RSI_DRIVER_LOCAL bool ReadTimeoutParameter(
    const char * name, std::chrono::milliseconds fallback, std::chrono::milliseconds & value);

  struct InitSequenceReport
  {
    bool sequence_complete = false;
    bool ok = false;
    std::string reason = "";
  };

  struct InterfaceData
  {
    std::vector<double> position_states;
    // Previous cycle's positions, kept only to derive velocity when the RSI XML
    // config maps no VELOCITY field. NaN means "no usable previous sample".
    std::vector<double> prev_position_states;
    std::vector<double> velocity_states;
    std::vector<double> torque_states;
    std::vector<double> gpio_states;
    std::vector<double> position_commands;
    std::vector<double> velocity_commands;
    std::vector<double> torque_commands;
    std::vector<double> gpio_commands;
  };

  struct OptionalInterfaceFlags
  {
    bool has_velocity_state_interface = false;
    bool has_torque_state_interface = false;
    bool has_velocity_command_interface = false;
    bool has_torque_command_interface = false;
  };

  struct EventState
  {
    double server_state =
      static_cast<double>(kuka_drivers_core::HardwareEvent::HARDWARE_EVENT_UNSPECIFIED);
    kuka_drivers_core::HardwareEvent last_event =
      kuka_drivers_core::HardwareEvent::HARDWARE_EVENT_UNSPECIFIED;
    std::mutex event_mutex;
    // The cause logged at the stop; held through INACTIVE, reset by a successful activation.
    std::atomic<kuka_drivers_core::StopReason> stop_reason{kuka_drivers_core::StopReason::NONE};
    // The KRC's stop message, set from the SDK's error event; the SDK reports no release, so it
    // holds until a successful activation, which the KRC refuses while the message stands.
    std::atomic<bool> drive_fault{false};
    double drive_fault_state = 0.0;
    // The driver's own RSI stop (a receive or send failure, or a stop no other slot explains),
    // the active source of inhibitor/rsi; held until a successful activation.
    std::atomic<bool> rsi_fault{false};
    // Latches requested from the SDK and mxA threads, one bit per slot; read() folds them in.
    std::atomic<uint8_t> latch_requests{0};
    // The exported slot values, written on the control thread only.
    std::array<double, INHIBITOR_COUNT> inhibitor_active{};
    std::array<double, INHIBITOR_COUNT> inhibitor_latched{};
  };

  struct RuntimeState
  {
    std::vector<int> gpio_states_to_commands_map;
    // Read by the mxA status thread, written by the lifecycle thread.
    std::atomic<bool> is_active{false};
    bool msg_received = false;
    // Set by Read() on a packet, cleared by Write() once it is answered; a commanded deactivate
    // finds it clear and puts the stop flag on the next packet, a fault's on the pending one.
    bool packet_unanswered = false;
    // Set by on_deactivate, cleared by read() once the cancel is confirmed and the drives are off.
    // Read by the mxA status thread: a guard stop inside this window is the stop's own doing.
    std::atomic<bool> stop_pending{false};
    bool cancel_confirmed = false;
    // Counts inactive read() calls (not RSI cycles) while a stop is pending.
    uint64_t stop_pending_reads = 0;
    std::chrono::steady_clock::time_point stop_requested_time{};
    bool stop_timeout_reported = false;
  };

  struct DiagnosticsState
  {
    std::chrono::steady_clock::time_point last_msg_received_time{};
    uint64_t packet_loss_count = 0;
    uint64_t last_ipoc = 0;
  };

  struct ControlState
  {
    StatusManager status_manager;
  };

  // Test-only gpio commands, exported only when the fault_injection parameter is true.
  struct FaultInjectionState
  {
    bool enabled = false;
    double stop_request = 0.0;
    double rsi_mute_ms = 0.0;
    double last_stop_request = 0.0;
    double last_rsi_mute_ms = 0.0;
    hardware_interface::CommandInterface::SharedPtr stop_request_interface;
    hardware_interface::CommandInterface::SharedPtr rsi_mute_ms_interface;
    bool muted = false;
    double mute_ms = 0.0;
    std::chrono::steady_clock::time_point mute_until{};
  };

  // Methods common for EKI and MXA version
  CallbackReturn extended_activation(const rclcpp_lifecycle::State &);
  CallbackReturn extended_deactivation(const rclcpp_lifecycle::State &);

  const rclcpp::Logger logger_;

  std::unique_ptr<kuka::external::control::kss::rsi::Robot> robot_ptr_;

  InterfaceData interface_data_;
  // The exported interfaces own their values; each is paired with the value it mirrors.
  std::vector<std::pair<hardware_interface::StateInterface::SharedPtr, const double *>>
    exported_states_;
  std::vector<std::pair<hardware_interface::CommandInterface::SharedPtr, double *>>
    exported_commands_;
  OptionalInterfaceFlags optional_interface_flags_;
  EventState event_state_;
  RuntimeState runtime_state_;
  DiagnosticsState diagnostics_state_;
  ControlState control_state_;
  FaultInjectionState fault_injection_;

  static constexpr std::chrono::milliseconds IDLE_SLEEP_DURATION{2};
  static constexpr std::chrono::milliseconds INIT_WAIT_DURATION{100};
  static constexpr std::chrono::milliseconds DRIVES_POWERED_CHECK_INTERVAL{100};

  // Bound on the wait for a pending cancel before on_cleanup releases the client.
  static constexpr std::chrono::milliseconds CLEANUP_CANCEL_TIMEOUT{1'000};
  static constexpr std::chrono::milliseconds CLEANUP_CANCEL_CHECK_INTERVAL{10};

  static constexpr std::chrono::milliseconds FIRST_STATUS_CHECK_INTERVAL{10};

  // Bounds on dropping the packets queued before an RSI start.
  static constexpr std::chrono::milliseconds DISCARD_RECEIVE_TIMEOUT{1};
  static constexpr uint64_t MAX_DISCARDED_PACKETS{1'000};

  // Timeouts from the *_timeout_ms hardware parameters.
  static constexpr std::chrono::milliseconds DEFAULT_CONFIGURE_TIMEOUT{1'500};
  static constexpr std::chrono::milliseconds DEFAULT_DRIVES_ON_TIMEOUT{3'000};
  static constexpr std::chrono::milliseconds DEFAULT_FIRST_PACKET_TIMEOUT{5'000};
  static constexpr std::chrono::milliseconds DEFAULT_STOP_PENDING_TIMEOUT{3'000};
  static constexpr std::chrono::milliseconds DEFAULT_READ_TIMEOUT{12};
  static constexpr std::chrono::milliseconds DEFAULT_FIRST_STATUS_TIMEOUT{1'000};
  std::chrono::milliseconds configure_timeout_{DEFAULT_CONFIGURE_TIMEOUT};
  std::chrono::milliseconds drives_on_timeout_{DEFAULT_DRIVES_ON_TIMEOUT};
  std::chrono::milliseconds first_packet_timeout_{DEFAULT_FIRST_PACKET_TIMEOUT};
  std::chrono::milliseconds stop_pending_timeout_{DEFAULT_STOP_PENDING_TIMEOUT};
  std::chrono::milliseconds read_timeout_{DEFAULT_READ_TIMEOUT};
  std::chrono::milliseconds first_status_timeout_{DEFAULT_FIRST_STATUS_TIMEOUT};

  // Fixed cycle time and control mode for this driver version.
  static constexpr RsiCycleTime RSI_CYCLE_TIME = RsiCycleTime::RSI_4MS;
  static constexpr double RSI_CYCLE_TIME_MS = 4.0;
  static constexpr kuka::external::control::ControlMode CONTROL_MODE =
    kuka::external::control::ControlMode::JOINT_POSITION_CONTROL;

  // threshold for logging slow RSI responses
  static constexpr std::chrono::milliseconds kWarningThreshold{2};

  // The SDK's error text for mxA error 801 (mxa/client.cc); keep in sync.
  static constexpr const char * STOP_MESSAGE_ERROR_TEXT = "STOPMESS active";

  // Mirrors kuka::external::control::kss::rsi::Robot::kStopReceiveTimeoutMs
  // (rsi/robot_interface.h), how long the SDK's StopControlling() waits for the packet it answers;
  // keep in sync.
  static constexpr std::chrono::milliseconds SDK_RSI_STOP_RECEIVE_TIMEOUT{40};

  // Interface prefix for state and command interfaces, enabling multi-robot support
  std::string interface_prefix_;

private:
  KUKA_RSI_DRIVER_LOCAL void ConfigureJoints(
    kuka::external::control::kss::Configuration & config) const;

  KUKA_RSI_DRIVER_LOCAL bool LoadXmlConfig(
    const std::string & path, kuka::external::control::kss::Configuration & config) const;

  // Parsed XML config populated during on_init; applied to Configuration in SetupRobot.
  std::optional<kuka::external::control::kss::MotionStateXmlConfiguration> motion_state_xml_config_;
  std::optional<kuka::external::control::kss::ControlSignalXmlConfiguration>
    control_signal_xml_config_;

  static constexpr std::string_view kTypeParamValue = "type";
  static constexpr std::string_view kIsExternalParamValue = "is_external";
  static constexpr std::string_view kRsiXmlConfigFileParam = "rsi_xml_config_file";
};
}  // namespace kuka_rsi_driver

#endif  // KUKA_RSI_DRIVER__HARDWARE_INTERFACE_RSI_BASE_HPP_
