# Copyright 2026 KUKA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Shared pieces for launch tests that inject an RSI fault while the component is active."""

import os
import re
import sys
import xml.etree.ElementTree as ET

import launch
import launch_ros.actions
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import DynamicInterfaceGroupValues
from controller_manager_msgs.srv import (
    ListControllers,
    SetHardwareComponentState,
    SwitchController,
)
from lifecycle_msgs.msg import State
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mxa_status_fake_support import (  # noqa: E402
    COMPONENT,
    OP_MODE_EXT,
    SIMULATOR_READ_TIMEOUT_MS,
    LogCursor,
    MxaStatusFakeTestCase,
    robot_description,
)

JTC = "joint_trajectory_controller"
JSB = "joint_state_broadcaster"
STOP_REASON_BROADCASTER = "stop_reason_broadcaster"

# Values of the state/stop_reason interface (kuka_drivers_core::StopReason).
STOP_REASON_NONE = 0.0
STOP_REASON_RSI_RECEIVE_FAILED = 1.0
STOP_REASON_RSI_SEND_FAILED = 2.0

SIMULATOR_NODE = "/kuka_rsi_simulator"
SIMULATOR_STATE_TOPIC = "/rsi_simulator_node/rsi/state"
SIMULATOR_COMMAND_TOPIC = "/rsi_simulator_node/rsi/command"
# The simulator starts silent, so no packets queue before activation; start_streaming resumes it.
SIMULATOR_SILENCE_PACKETS = 1000000
# What the simulator logs for every command it receives.
SIMULATOR_COMMAND_LINE = re.compile(r"msg: b'([^'\n]*)'")

RECEIVE_FAILURE_LINE = re.compile(
    r"Stopping: no RSI packet within (\d+) ms of the last one, IPOC (\d+)"
)
SEND_FAILURE_LINE = re.compile(r"Stopping: sending the RSI command for IPOC (\d+) failed")
JTC_DEACTIVATED_LINE = (
    f"Deactivating controllers [{JTC} ] as their command interfaces are tied to "
    "DEACTIVATEing hardware components"
)
NO_STOP_PACKET_LINE = "Message not received, but stop requested. Cancelling RSI program."


def _config(name):
    return os.path.join(get_package_share_directory("kuka_rsi_driver"), "config", name)


def _test_config(name):
    return os.path.join(get_package_share_directory("kuka_rsi_driver"), "test", "config", name)


def _spawner(name, param_file=None, inactive=False):
    arguments = [name, "-c", "controller_manager"]
    if param_file:
        arguments += ["--param-file", param_file]
    if inactive:
        arguments.append("--inactive")
    return launch_ros.actions.Node(
        package="controller_manager", executable="spawner", output="screen", arguments=arguments
    )


def fault_launch_description(read_timeout_ms=SIMULATOR_READ_TIMEOUT_MS):
    """control_node on the fake with JSB active, JTC inactive and the stop_reason broadcaster."""
    description = {"robot_description": robot_description(read_timeout_ms=read_timeout_ms)}

    control_node = launch_ros.actions.Node(
        package="kuka_drivers_core",
        executable="control_node",
        output="screen",
        parameters=[
            description,
            _test_config("fault_latch_controller_manager.yaml"),
            {"hardware_components_initial_state": {"inactive": [COMPONENT]}},
        ],
    )

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[description],
    )

    simulator = launch_ros.actions.Node(
        package="kuka_rsi_simulator",
        executable="rsi_simulator",
        name="kuka_rsi_simulator",
        parameters=[{"fault_dropped_packet_count": SIMULATOR_SILENCE_PACKETS}],
    )

    return launch.LaunchDescription(
        [
            control_node,
            robot_state_publisher,
            simulator,
            _spawner(JSB),
            _spawner(JTC, _config("joint_trajectory_controller_config.yaml"), inactive=True),
            _spawner(STOP_REASON_BROADCASTER, _test_config("stop_reason_broadcaster.yaml")),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def _ipoc(xml_text):
    return int(ET.fromstring(xml_text).find("IPOC").text)


def _stop_flag(xml_text):
    node = ET.fromstring(xml_text).find("Stop")
    return node is not None and node.text == "1"


class RsiFaultTestCase(MxaStatusFakeTestCase):
    """MxaStatusFakeTestCase that tracks stop_reason and the simulator's IPOC traffic."""

    def setUp(self):
        super().setUp()
        self.stop_reason_value = None
        self.state_ipocs = []
        self.commands = []
        self.node.create_subscription(
            DynamicInterfaceGroupValues,
            f"/{STOP_REASON_BROADCASTER}/gpio_states",
            self._on_gpio_states,
            10,
        )
        self.node.create_subscription(
            String,
            SIMULATOR_STATE_TOPIC,
            lambda msg: self.state_ipocs.append(_ipoc(msg.data)),
            100,
        )
        self.node.create_subscription(
            String,
            SIMULATOR_COMMAND_TOPIC,
            lambda msg: self.commands.append((_ipoc(msg.data), _stop_flag(msg.data))),
            100,
        )

    def _on_gpio_states(self, msg):
        for group, values in zip(msg.interface_groups, msg.interface_values):
            if group == "state" and "stop_reason" in values.interface_names:
                self.stop_reason_value = values.values[values.interface_names.index("stop_reason")]

    def stop_reason(self):
        self.stop_reason_value = None
        self.assertTrue(
            self._spin_until(lambda: self.stop_reason_value is not None, timeout=5),
            "no stop_reason published",
        )
        return self.stop_reason_value

    def controller_state(self, name):
        result = self._call(
            ListControllers,
            "/controller_manager/list_controllers",
            ListControllers.Request(),
            timeout=30,
        )
        states = {c.name: c.state for c in result.controller}
        self.assertTrue(name in states, states)
        return states[name]

    def switch(self, activate=(), deactivate=()):
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        result = self._call(
            SwitchController, "/controller_manager/switch_controller", request, timeout=20
        )
        self.assertTrue(result.ok, f"switch_controller {activate} {deactivate} refused")

    def set_simulator_fault(self, name, count):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name=name,
                value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=count),
            )
        ]
        result = self._call(SetParameters, f"{SIMULATOR_NODE}/set_parameters", request, 10)
        self.assertTrue(result.results[0].successful, result.results[0].reason)

    def resume_and_activate(self, log, seconds=1.0):
        """
        Activate with the simulator resumed only once the request is in flight.

        The simulator reads one reply per cycle, so two packets queued before the first read
        would leave it a cycle behind for the rest of the run.
        """
        self.inject(f"operation_mode={OP_MODE_EXT} drives_powered=1 motion_possible=1")
        request = SetHardwareComponentState.Request()
        request.name = COMPONENT
        request.target_state.id = State.PRIMARY_STATE_ACTIVE
        client = self.node.create_client(
            SetHardwareComponentState, "/controller_manager/set_hardware_component_state"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=30), "no component state service")
        future = client.call_async(request)
        self.assertTrue(log.wait_for("Starting joint position control at 4 ms", 10), "no start")
        self.set_simulator_fault("fault_dropped_packet_count", 0)
        self.assertTrue(self._wait_future(future, 60).ok, "activate refused")
        self.assertTrue(log.wait_for(f"Successful 'activate' of hardware '{COMPONENT}'", 30))
        self._spin_for(seconds)

    def start_streaming(self, log):
        """Bring the component up, activate JTC and let a few RSI cycles stream."""
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()
        self.resume_and_activate(log)
        self.switch(activate=[JTC])
        self._spin_for(0.5)
        self.assertEqual(self.controller_state(JTC), "active")
        self.assertEqual(self.controller_state(JSB), "active")
        self.assertEqual(self.controller_state(STOP_REASON_BROADCASTER), "active")
        self.assertEqual(self.stop_reason(), STOP_REASON_NONE)

    def assert_stopped(self, log, reason):
        """Check the component went INACTIVE on its own, took JTC with it and left JSB running."""
        self.assertTrue(log.wait_for(JTC_DEACTIVATED_LINE, timeout=5), "JTC not deactivated")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)
        self.assertEqual(self.controller_state(JTC), "inactive")
        self.assertEqual(self.controller_state(JSB), "active")
        self.assertEqual(self.controller_state(STOP_REASON_BROADCASTER), "active")
        self.assertEqual(self.stop_reason(), reason)

    def wait_for_states_after(self, ipoc, count, timeout=10):
        """Wait until the simulator has sent `count` states with an IPOC above `ipoc`."""
        self.assertTrue(
            self._spin_until(
                lambda: sum(1 for i in self.state_ipocs if i > ipoc) >= count, timeout
            ),
            f"simulator sent fewer than {count} states after IPOC {ipoc}: {self.state_ipocs}",
        )

    def commands_after(self, ipoc):
        return [(i, stop) for i, stop in self.commands if i > ipoc]

    def logged_commands(self, log):
        """(IPOC, stop flag) of every command the simulator logged since the cursor."""
        return [
            (_ipoc(match.group(1)), _stop_flag(match.group(1)))
            for match in log.matches(SIMULATOR_COMMAND_LINE)
        ]


__all__ = [
    "COMPONENT",
    "JSB",
    "JTC",
    "JTC_DEACTIVATED_LINE",
    "NO_STOP_PACKET_LINE",
    "OP_MODE_EXT",
    "RECEIVE_FAILURE_LINE",
    "SEND_FAILURE_LINE",
    "SIMULATOR_READ_TIMEOUT_MS",
    "SIMULATOR_SILENCE_PACKETS",
    "STOP_REASON_BROADCASTER",
    "STOP_REASON_NONE",
    "STOP_REASON_RSI_RECEIVE_FAILED",
    "STOP_REASON_RSI_SEND_FAILED",
    "LogCursor",
    "RsiFaultTestCase",
    "fault_launch_description",
]
