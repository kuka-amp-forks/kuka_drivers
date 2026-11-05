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
from kuka_driver_interfaces.msg import HardwareEvent
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
INHIBITOR_BROADCASTER = "inhibitor_broadcaster"
EVENT_BROADCASTER = "event_broadcaster"

# kuka_drivers_core::HardwareEvent::ERROR, as published on the event broadcaster's topic.
HARDWARE_EVENT_ERROR = 6

# The inhibitor slots (gpio groups inhibitor/<name>), their interfaces and fixed operator_action.
INHIBITORS = ("estop", "guard", "mode", "drive_fault", "brake_test", "rsi")
SLOT_INTERFACES = ("active", "latched", "operator_action")
OPERATOR_ACTION = {
    "estop": 1.0,
    "guard": 1.0,
    "mode": 1.0,
    "drive_fault": 1.0,
    "brake_test": 1.0,
    "rsi": 0.0,
}

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
STATUS_STOP_LINE = re.compile(
    r"Stopping: ([a-zA-Z ]+) \((emergency_stop=\d guard_stop=\d drives_powered=\d "
    r"motion_possible=\d operation_mode=\d+), IPOC (\d+)\)"
)
# A new e-stop/guard stop while INACTIVE, latched from the mxA thread itself.
INACTIVE_LATCH_LINE = re.compile(
    r"Latched while inactive: inhibitor/estop=(\d) inhibitor/guard=(\d) "
    r"\(emergency_stop=(\d) guard_stop=(\d)\)"
)
# The slots at the instant the component leaves ACTIVE on a fault.
INHIBITORS_AT_STOP_LINE = re.compile(
    r"Inhibitors at the stop: estop=(\d) guard=(\d) mode=(\d) drive_fault=(\d) "
    r"brake_test=(\d) rsi=(\d)"
)
# A stop with no slot active: the command link is lost, inhibitor/rsi is raised.
UNATTRIBUTED_LINE = re.compile(
    r"Stopping: '([^']+)' with no inhibitor active, the command link to the controller is "
    r"lost; raising inhibitor/rsi"
)
MXA_LOSS_LINE = re.compile(
    r"Stopping: no mxA telegram within (\d+) ms, the newest is (\d+) ms old"
)
MXA_RELEASED_LINE = "Releasing the mxA client, the stop cause is 'mxA connection lost'"
# An SDK stop or error event latched while active.
CONTROL_EVENT_STOP_LINE = re.compile(
    r"Stopping: (external control stopped|external control error): (\S+)"
)
# What the event observer logs for any SDK error event, latched or not.
SDK_ERROR_LINE = re.compile(r"External control stopped due to an error: ([^\n]+)")
SDK_RECEIVE_TIMEOUT_TEXT = "Keep-alive thread: UDP subscriber receive timed out"
# What on_deactivate logs once its requests are out, with or without a stop packet.
DEACTIVATION_REQUESTS_LINE = "Deactivation requests issued"
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


def fault_launch_description(
    read_timeout_ms=SIMULATOR_READ_TIMEOUT_MS,
    respawn_simulator=False,
    inactive_controllers=(),
    gpio_config="",
):
    """
    control_node on the fake: JTC inactive; JSB, inhibitor and event broadcasters active.

    The simulator exits on the stop flag, so a test that reactivates after a stop packet
    needs it respawned; it comes back silent, and resume_and_activate resumes it.
    inactive_controllers are (name, file in test/config) pairs, loaded inactive.
    gpio_config is a xacro file with the gpio_config macro; empty leaves the gpio empty.
    """
    description = {
        "robot_description": robot_description(
            read_timeout_ms=read_timeout_ms, gpio_config=gpio_config
        )
    }

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
        respawn=respawn_simulator,
        respawn_delay=0.2,
    )

    return launch.LaunchDescription(
        [
            control_node,
            robot_state_publisher,
            simulator,
            _spawner(JSB),
            _spawner(JTC, _config("joint_trajectory_controller_config.yaml"), inactive=True),
            _spawner(INHIBITOR_BROADCASTER, _test_config("inhibitor_broadcaster.yaml")),
            _spawner(EVENT_BROADCASTER, _config("kuka_event_broadcaster_config.yaml")),
            *[
                _spawner(name, _test_config(param_file), inactive=True)
                for name, param_file in inactive_controllers
            ],
            launch_testing.actions.ReadyToTest(),
        ]
    )


def _ipoc(xml_text):
    return int(ET.fromstring(xml_text).find("IPOC").text)


def _stop_flag(xml_text):
    node = ET.fromstring(xml_text).find("Stop")
    return node is not None and node.text == "1"


class RsiFaultTestCase(MxaStatusFakeTestCase):
    """MxaStatusFakeTestCase that tracks the slots, hardware events and the simulator's IPOCs."""

    def setUp(self):
        super().setUp()
        self.inhibitor_values = None
        self.state_values = {}
        self.hardware_events = []
        self.state_ipocs = []
        self.commands = []
        self.node.create_subscription(
            HardwareEvent,
            f"/{EVENT_BROADCASTER}/hardware_event",
            lambda msg: self.hardware_events.append(msg.event),
            10,
        )
        self.node.create_subscription(
            DynamicInterfaceGroupValues,
            f"/{INHIBITOR_BROADCASTER}/gpio_states",
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
        slots = {}
        for group, values in zip(msg.interface_groups, msg.interface_values):
            if group == "state":
                self.state_values = dict(zip(values.interface_names, values.values))
            elif group.startswith("inhibitor/"):
                slots[group[len("inhibitor/") :]] = dict(  # noqa: E203
                    zip(values.interface_names, values.values)
                )
        if slots:
            self.inhibitor_values = slots

    def inhibitors(self):
        """Return the next published slots: {name: {active, latched, operator_action}}."""
        self.inhibitor_values = None
        self.assertTrue(
            self._spin_until(lambda: self.inhibitor_values is not None, timeout=5),
            "no inhibitor slots published",
        )
        self.assertEqual(set(self.inhibitor_values), set(INHIBITORS), self.inhibitor_values)
        return self.inhibitor_values

    def active(self):
        return {name for name, slot in self.inhibitors().items() if slot["active"] == 1.0}

    def latched(self):
        return {name for name, slot in self.inhibitors().items() if slot["latched"] == 1.0}

    def assert_active(self, expected):
        self.assertEqual(self.active(), set(expected))

    def assert_latched(self, expected):
        self.assertEqual(self.latched(), set(expected))

    def state_value(self, name):
        """Return the next published value of the state/<name> interface."""
        self.state_values = {}
        self.assertTrue(
            self._spin_until(lambda: name in self.state_values, timeout=5),
            f"no state/{name} published",
        )
        return self.state_values[name]

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

    def ensure_controllers_active(self, *names):
        inactive = [name for name in names if self.controller_state(name) != "active"]
        if inactive:
            self.switch(activate=inactive)

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
        self.assertEqual(self.controller_state(INHIBITOR_BROADCASTER), "active")
        self.assert_latched(set())

    def assert_stopped(self, log, latched):
        """
        Check the component went INACTIVE on its own, took JTC with it and left JSB running.

        latched names the slots the stop must have latched, and no other.
        """
        self.assertTrue(log.wait_for(JTC_DEACTIVATED_LINE, timeout=5), "JTC not deactivated")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)
        self.assertEqual(self.controller_state(JTC), "inactive")
        self.assertEqual(self.controller_state(JSB), "active")
        self.assertEqual(self.controller_state(INHIBITOR_BROADCASTER), "active")
        self.assertIsNotNone(
            log.wait_for_match(INHIBITORS_AT_STOP_LINE, timeout=5), "no inhibitors-at-stop line"
        )
        self.assert_latched(latched)

    def wait_for_component_state(self, state_id, timeout=10):
        self.assertTrue(
            self._spin_until(lambda: self.component_state() == state_id, timeout),
            f"component did not reach state {state_id}, it is {self.component_state()}",
        )

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
    "CONTROL_EVENT_STOP_LINE",
    "DEACTIVATION_REQUESTS_LINE",
    "EVENT_BROADCASTER",
    "HARDWARE_EVENT_ERROR",
    "INACTIVE_LATCH_LINE",
    "INHIBITOR_BROADCASTER",
    "INHIBITORS",
    "INHIBITORS_AT_STOP_LINE",
    "JSB",
    "JTC",
    "JTC_DEACTIVATED_LINE",
    "MXA_LOSS_LINE",
    "MXA_RELEASED_LINE",
    "NO_STOP_PACKET_LINE",
    "OP_MODE_EXT",
    "OPERATOR_ACTION",
    "RECEIVE_FAILURE_LINE",
    "SDK_ERROR_LINE",
    "SDK_RECEIVE_TIMEOUT_TEXT",
    "SEND_FAILURE_LINE",
    "SIMULATOR_READ_TIMEOUT_MS",
    "SIMULATOR_SILENCE_PACKETS",
    "SLOT_INTERFACES",
    "STATUS_STOP_LINE",
    "UNATTRIBUTED_LINE",
    "LogCursor",
    "RsiFaultTestCase",
    "fault_launch_description",
]
