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

"""The state/* interfaces carry the live controller status; drive_fault latches the SDK's 801."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from controller_manager_msgs.srv import ListHardwareInterfaces
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mxa_status_fake_support import OP_MODE_EXT, OP_MODE_T1  # noqa: E402
from rsi_fault_support import (  # noqa: E402
    CONTROL_EVENT_STOP_LINE,
    JTC,
    SDK_ERROR_LINE,
    STOP_REASON_CONTROL_ERROR,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

# The nine mxA flags the status manager exports, plus the drive fault.
STATUS_INTERFACES = (
    "control_mode",
    "cycle_time",
    "drives_powered",
    "emergency_stop",
    "guard_stop",
    "in_motion",
    "motion_possible",
    "operation_mode",
    "robot_stopped",
    "drive_fault",
)
# The SDK's error text for mxA error 801; the fake delivers '+' as a space.
STOP_MESSAGE_ERROR = "STOPMESS+active"
DRIVE_FAULT_LINE = "Drive fault: the KRC latched a stop message (mxA error 801)"
STOP_MESSAGE_STOP_LINE = "Stopping: external control error: STOPMESS active"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(respawn_simulator=True)


class TestExportsStatusStateInterfaces(RsiFaultTestCase):
    def setUp(self):
        super().setUp()
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()

    def test_drive_fault_latches_from_sdk_error(self, proc_output):
        # While INACTIVE the error sets the flag and latches no stop.
        held = self.stop_reason()
        log = LogCursor(proc_output)
        self.inject(f"error={STOP_MESSAGE_ERROR}")
        self.assertTrue(log.wait_for(DRIVE_FAULT_LINE, timeout=5), "no drive fault line")
        self.assertEqual(self.state_value("drive_fault"), 1.0)
        self.assertEqual(log.matches(CONTROL_EVENT_STOP_LINE), [], "stop latched while inactive")
        self.assertEqual(self.stop_reason(), held)

        # Other errors leave it set, a repeat logs nothing new, a refused activation keeps it.
        log = LogCursor(proc_output)
        self.inject("error=Keep-alive+thread+error+occurred+with+ID:+529")
        self.assertIsNotNone(log.wait_for_match(SDK_ERROR_LINE, timeout=5), "no error event")
        self.assertEqual(self.state_value("drive_fault"), 1.0)
        self.inject(f"error={STOP_MESSAGE_ERROR}")
        self.assertFalse(log.wait_for(DRIVE_FAULT_LINE, timeout=0.5), "drive fault logged twice")
        self.inject(f"operation_mode={OP_MODE_T1}")
        self.assertFalse(self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activated in T1")
        self.assertEqual(self.state_value("drive_fault"), 1.0)

        # A successful activation clears it.
        log = LogCursor(proc_output)
        self.resume_and_activate(log)
        self.assertEqual(self.state_value("drive_fault"), 0.0)

        # While ACTIVE the error stops the component and sets the flag again.
        self.switch(activate=[JTC])
        log = LogCursor(proc_output)
        self.inject(f"error={STOP_MESSAGE_ERROR}")
        self.assertTrue(log.wait_for(STOP_MESSAGE_STOP_LINE, timeout=5), "no stop line")
        self.assert_stopped(log, STOP_REASON_CONTROL_ERROR)
        self.assertEqual(self.state_value("drive_fault"), 1.0)
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)

        log = LogCursor(proc_output)
        self.resume_and_activate(log)
        self.assertEqual(self.state_value("drive_fault"), 0.0)
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)

    def test_exports_status_state_interfaces(self, proc_output):
        result = self._call(
            ListHardwareInterfaces,
            "/controller_manager/list_hardware_interfaces",
            ListHardwareInterfaces.Request(),
            timeout=30,
        )
        states = [i.name for i in result.state_interfaces]
        for name in STATUS_INTERFACES:
            self.assertTrue(f"state/{name}" in states, f"state/{name} not exported: {states}")

        # Live, not latched: the flags the component does not stop on while INACTIVE follow the
        # fake both ways.
        self.inject(f"operation_mode={OP_MODE_T1} in_motion=1 robot_stopped=0 motion_possible=0")
        self.assertEqual(self.state_value("operation_mode"), OP_MODE_T1)
        self.assertEqual(self.state_value("in_motion"), 1.0)
        self.assertEqual(self.state_value("robot_stopped"), 0.0)
        self.assertEqual(self.state_value("motion_possible"), 0.0)
        self.assertEqual(self.state_value("drive_fault"), 0.0)
        self.inject(f"operation_mode={OP_MODE_EXT} in_motion=0 robot_stopped=1 motion_possible=1")
        self.assertEqual(self.state_value("operation_mode"), OP_MODE_EXT)
        self.assertEqual(self.state_value("in_motion"), 0.0)
        self.assertEqual(self.state_value("robot_stopped"), 1.0)
        self.assertEqual(self.state_value("motion_possible"), 1.0)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
