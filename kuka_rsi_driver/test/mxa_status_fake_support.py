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

"""Shared pieces for launch tests that run the mxA component on the test-only status fake."""

import os
import socket
import sys
import time

import xacro
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListHardwareComponents, SetHardwareComponentState
from lifecycle_msgs.msg import State
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from joint_move_test_case import JointMoveTestCase  # noqa: E402

COMPONENT = "kr6_r700_2"
FAKE_PORT = 59153
OP_MODE_T1 = 1
OP_MODE_EXT = 4

PRODUCTION_PLUGIN = "<plugin>kuka_rsi_driver::KukaMxaRsiHardwareInterface</plugin>"
FAKE_PLUGIN = (
    "<plugin>kuka_rsi_driver::KukaMxaRsiStatusFakeHardwareInterface</plugin>"
    f'<param name="mxa_status_fake_port">{FAKE_PORT}</param>'
)


def robot_description():
    """Build the mxa_rsi description with the test-only plugin in place of the shipped one."""
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
    assert PRODUCTION_PLUGIN in urdf
    return urdf.replace(PRODUCTION_PLUGIN, FAKE_PLUGIN)


class LogCursor:
    """Matches only process output produced after the cursor was taken."""

    def __init__(self, proc_output):
        self._proc_output = proc_output
        self._start = len(list(proc_output))

    def wait_for(self, text, timeout):
        end = time.monotonic() + timeout
        while True:
            new_events = list(self._proc_output)[self._start :]  # noqa: E203
            if any(text in event.text.decode(errors="replace") for event in new_events):
                return True
            if time.monotonic() > end:
                return False
            time.sleep(0.05)


class MxaStatusFakeTestCase(JointMoveTestCase):
    """JointMoveTestCase with a socket into the fake and component state helpers."""

    # Cycles of read() at 250 Hz copy a delivered status into the state interfaces.
    SETTLE_S = 0.5

    def setUp(self):
        super().setUp()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.sock.close()
        super().tearDown()

    def inject(self, datagram):
        self.sock.sendto(datagram.encode(), ("127.0.0.1", FAKE_PORT))
        self._spin_for(self.SETTLE_S)

    def component_state(self):
        result = self._call(
            ListHardwareComponents,
            "/controller_manager/list_hardware_components",
            ListHardwareComponents.Request(),
            timeout=30,
        )
        states = {c.name: c.state.id for c in result.component}
        self.assertTrue(COMPONENT in states, states)
        return states[COMPONENT]

    def set_component_state(self, state_id):
        request = SetHardwareComponentState.Request()
        request.name = COMPONENT
        request.target_state.id = state_id
        result = self._call(
            SetHardwareComponentState,
            "/controller_manager/set_hardware_component_state",
            request,
            timeout=60,
        )
        return result.ok

    def ensure_component_state(self, state_id):
        if self.component_state() != state_id:
            self.assertTrue(self.set_component_state(state_id), f"transition to {state_id}")
        self.assertEqual(self.component_state(), state_id)

    def deactivate_and_wait_for_simulator(self, proc_info):
        """Deactivate, then wait for the RSI simulator to exit on the stop flag it reads late."""
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        proc_info.assertWaitForShutdown(process="rsi_simulator", timeout=30)

    def publish_is_configured(self):
        """Tell control_node to run read() and write(): it only does so after this is true."""
        publisher = self.node.create_publisher(
            Bool,
            "/robot_manager/is_configured",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.assertTrue(
            self._spin_until(lambda: publisher.get_subscription_count() > 0, timeout=10),
            "control_node is not subscribed to /robot_manager/is_configured",
        )
        for _ in range(5):
            publisher.publish(Bool(data=True))
            self._spin_for(0.1)
        self.node.destroy_publisher(publisher)


__all__ = [
    "COMPONENT",
    "FAKE_PORT",
    "OP_MODE_EXT",
    "OP_MODE_T1",
    "LogCursor",
    "MxaStatusFakeTestCase",
    "robot_description",
]
