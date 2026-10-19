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

"""Injected mxA status drives the component's activation and read() checks, and can stall."""

import os
import socket
import sys
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import SetHardwareComponentState
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import State
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from joint_move_test_case import JointMoveTestCase  # noqa: E402

COMPONENT = "kr6_r700_2"
FAKE_PORT = 59153
OP_MODE_T1 = 1
OP_MODE_EXT = 4
# Cycles of read() at 250 Hz copy a delivered status into the state interfaces.
SETTLE_S = 0.5


def _robot_description():
    urdf = xacro.process_file(
        os.path.join(
            get_package_share_directory("kuka_agilus_support"), "urdf", f"{COMPONENT}.urdf.xacro"
        ),
        mappings={
            "driver_version": "mxa_rsi",
            "client_ip": "0.0.0.0",
            "client_port": "59152",
            "controller_ip": "127.0.0.1",
            "verify_robot_model": "true",
            "roundtrip_time": "4000",
        },
    ).toxml()
    anchor = '<param name="mxa_client_port">1337</param>'
    assert anchor in urdf
    return urdf.replace(anchor, anchor + f'<param name="mxa_status_fake_port">{FAKE_PORT}</param>')


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    robot_description = {"robot_description": _robot_description()}

    control_node = launch_ros.actions.Node(
        package="kuka_drivers_core",
        executable="control_node",
        output="screen",
        parameters=[
            robot_description,
            {
                "update_rate": 250,
                "hardware_components_initial_state": {"unconfigured": [COMPONENT]},
            },
        ],
    )

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("kuka_rsi_simulator"),
                "/launch/",
                "kuka_rsi_simulator.launch.py",
            ]
        )
    )

    return launch.LaunchDescription(
        [
            control_node,
            robot_state_publisher,
            simulator,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMxaStatusFake(JointMoveTestCase):
    def setUp(self):
        super().setUp()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.sock.close()
        super().tearDown()

    def _inject(self, datagram):
        self.sock.sendto(datagram.encode(), ("127.0.0.1", FAKE_PORT))
        self._spin_for(SETTLE_S)

    def _set_component_state(self, state_id, label):
        request = SetHardwareComponentState.Request()
        request.name = COMPONENT
        request.target_state.id = state_id
        request.target_state.label = label
        result = self._call(
            SetHardwareComponentState,
            "/controller_manager/set_hardware_component_state",
            request,
            timeout=60,
        )
        return result.ok

    def test_injected_status_drives_activation_and_read(self, proc_output):
        proc_output.assertWaitFor(
            f"Successful initialization of hardware '{COMPONENT}'", timeout=30
        )

        # Configure completes without an mxA server.
        self.assertTrue(self._set_component_state(State.PRIMARY_STATE_INACTIVE, "inactive"))
        proc_output.assertWaitFor(f"mxA status fake listening on 127.0.0.1:{FAKE_PORT}", timeout=5)
        proc_output.assertWaitFor(f"Successful 'configure' of hardware '{COMPONENT}'", timeout=30)

        # control_node runs read() and write() only after robot_manager/is_configured is true.
        is_configured = self.node.create_publisher(
            Bool,
            "/robot_manager/is_configured",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.assertTrue(
            self._spin_until(lambda: is_configured.get_subscription_count() > 0, timeout=10),
            "control_node is not subscribed to /robot_manager/is_configured",
        )
        for _ in range(5):
            is_configured.publish(Bool(data=True))
            self._spin_for(0.1)

        # E-stop refuses activation.
        self._inject(
            f"emergency_stop=1 operation_mode={OP_MODE_EXT} drives_powered=1 motion_possible=1"
        )
        self.assertFalse(self._set_component_state(State.PRIMARY_STATE_ACTIVE, "active"))
        proc_output.assertWaitFor("Emergency stop is active", timeout=5)

        # A non-EXT operation mode refuses activation.
        self._inject(f"emergency_stop=0 operation_mode={OP_MODE_T1} drives_powered=0")
        self.assertFalse(self._set_component_state(State.PRIMARY_STATE_ACTIVE, "active"))
        proc_output.assertWaitFor("KRC not in EXT", timeout=5)

        # A stalled stream withholds the EXT update, so activation stays refused.
        self._inject("stall")
        proc_output.assertWaitFor("mxA status fake: stream stalled", timeout=5)
        self._inject(f"operation_mode={OP_MODE_EXT}")
        self.assertFalse(self._set_component_state(State.PRIMARY_STATE_ACTIVE, "active"))

        # Resuming delivers it; activation turns the drives on and starts RSI on the simulator.
        self._inject("resume")
        self.assertTrue(self._set_component_state(State.PRIMARY_STATE_ACTIVE, "active"))
        proc_output.assertWaitFor("Drives successfully powered on", timeout=5)
        proc_output.assertWaitFor(f"Successful 'activate' of hardware '{COMPONENT}'", timeout=30)

        # While active, read() classifies each injected condition.
        self._inject("motion_possible=0")
        proc_output.assertWaitFor("Motion is not possible", timeout=5)
        self._inject("motion_possible=1")

        self._inject("emergency_stop=1")
        proc_output.assertWaitFor("Emergency stop detected!", timeout=5)
        self._inject("emergency_stop=0")


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
