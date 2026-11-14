# Copyright 2026 KUKA Hungaria Kft.
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

"""gpio/stop_request and gpio/rsi_mute_ms stop the component; it activates and streams again."""

import os
import re
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from controller_manager_msgs.srv import ListHardwareInterfaces
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    JSB,
    JTC,
    UNATTRIBUTED_LINE,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
    hardware_params,
)

FAULT_CONTROLLER = "fault_injection_controller"
MUTE_MS = 600
# The simulator's 40 ms cycle sends at least this many packets during the mute.
MIN_UNANSWERED = MUTE_MS // 40 // 2

STOP_REQUESTED_LINE = "Stopping: stop requested through gpio/stop_request"
MUTE_START_LINE = re.compile(r"Withholding RSI replies for (\d+) ms after IPOC (\d+)")
MUTE_STOP_LINE = re.compile(
    r"Stopping: RSI replies withheld for (\d+) ms through gpio/rsi_mute_ms"
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(
        respawn_simulator=True,
        inactive_controllers=[(FAULT_CONTROLLER, "fault_injection_controller.yaml")],
        extra_params=hardware_params(fault_injection="true"),
    )


class TestFaultInjection(RsiFaultTestCase):
    def setUp(self):
        super().setUp()
        self.fault_pub = self.node.create_publisher(
            DynamicInterfaceGroupValues, f"/{FAULT_CONTROLLER}/commands", 10
        )

    def command(self, name, value):
        msg = DynamicInterfaceGroupValues()
        msg.interface_groups = ["gpio"]
        msg.interface_values = [InterfaceValue(interface_names=[name], values=[float(value)])]
        for _ in range(3):
            self.fault_pub.publish(msg)
            self._spin_for(0.05)

    def assert_inactive_with_rsi(self, log):
        self.wait_for_component_state(State.PRIMARY_STATE_INACTIVE)
        self.assertTrue(
            self._spin_until(lambda: self.controller_state(JTC) == "inactive", timeout=5),
            "JTC still active",
        )
        self.assertEqual(self.controller_state(FAULT_CONTROLLER), "inactive")
        self.assertEqual(self.controller_state(JSB), "active")
        # The injected stop is the driver's own: inhibitor/rsi, not an unexplained stop.
        self.assertIsNone(log.wait_for_match(UNATTRIBUTED_LINE, timeout=0.1), "unattributed stop")
        self.assert_active({"rsi"})
        self.assert_latched({"rsi"})

    def assert_streaming(self):
        ipoc = max(self.state_ipocs, default=0)
        self.assertTrue(
            self._spin_until(lambda: len(self.commands_after(ipoc)) >= 3, timeout=5),
            f"no RSI commands after IPOC {ipoc}",
        )

    def test_exports_fault_injection_commands(self):
        result = self._call(
            ListHardwareInterfaces,
            "/controller_manager/list_hardware_interfaces",
            ListHardwareInterfaces.Request(),
            timeout=30,
        )
        commands = [i.name for i in result.command_interfaces]
        self.assertTrue("gpio/stop_request" in commands, commands)
        self.assertTrue("gpio/rsi_mute_ms" in commands, commands)

    def test_stop_request_and_rsi_mute(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)
        self.switch(activate=[FAULT_CONTROLLER])

        # stop_request: the stop packet ends the simulator, which comes back silent.
        log = LogCursor(proc_output)
        self.command("stop_request", 1)
        self.assertTrue(log.wait_for(STOP_REQUESTED_LINE, timeout=5), "no stop line")
        self.assert_inactive_with_rsi(log)
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)

        # stop_request was last written as 1 and the controller writes nothing after activation,
        # so a handle left at 1 would stop the component again on its first write().
        log = LogCursor(proc_output)
        self.resume_and_activate(log)
        self.assertFalse(log.wait_for(STOP_REQUESTED_LINE, timeout=0.1), "spurious stop request")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)
        self.assert_latched(set())
        self.switch(activate=[JTC, FAULT_CONTROLLER])
        self._spin_for(0.5)
        self.assertFalse(log.wait_for(STOP_REQUESTED_LINE, timeout=0.1), "spurious stop request")
        self.assert_streaming()
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)

        # rsi_mute_ms: nothing answers the simulator for the window, then the receive fails.
        log = LogCursor(proc_output)
        self.command("rsi_mute_ms", MUTE_MS)
        start = log.wait_for_match(MUTE_START_LINE, timeout=5)
        self.assertIsNotNone(start, "no mute start line")
        self.assertEqual(int(start.group(1)), MUTE_MS)
        muted_after = int(start.group(2))
        stop = log.wait_for_match(MUTE_STOP_LINE, timeout=5)
        self.assertIsNotNone(stop, "no mute stop line")
        self.assert_inactive_with_rsi(log)
        self.assertGreaterEqual(
            len([i for i in self.state_ipocs if i > muted_after]), MIN_UNANSWERED
        )
        self.assertEqual(self.commands_after(muted_after), [], "a muted packet was answered")
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")

        log = LogCursor(proc_output)
        self.resume_and_activate(log)
        self.assert_latched(set())
        self.switch(activate=[JTC])
        self.move_joint_and_wait()
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)

        self.switch(deactivate=[JTC])
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
