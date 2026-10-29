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

"""Each mxA status that forbids motion deactivates the component with its own stop_reason."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mxa_status_fake_support import OP_MODE_T1  # noqa: E402
from rsi_fault_support import (  # noqa: E402
    RECEIVE_FAILURE_LINE,
    STATUS_STOP_LINE,
    STOP_REASON_DRIVES_NOT_POWERED,
    STOP_REASON_EMERGENCY_STOP,
    STOP_REASON_GUARD_STOP,
    STOP_REASON_NONE,
    STOP_REASON_NOT_IN_EXT,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(respawn_simulator=True)


class TestMxaStatusDeactivates(RsiFaultTestCase):
    def stop_on_status(self, proc_output, datagram, reason, name, clear):
        """Stream, inject a status, check the stop names it, then clear it and reactivate."""
        log = LogCursor(proc_output)
        self.start_streaming(log)

        log = LogCursor(proc_output)
        self.inject(datagram)
        match = log.wait_for_match(STATUS_STOP_LINE, timeout=5)
        self.assertIsNotNone(match, "no stop line in the driver log")
        self.assertEqual(match.group(1), name)
        self.assertTrue(datagram in match.group(2), "raw status not logged with the cause")
        self.assert_stopped(log, reason)
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.assertEqual(log.matches(RECEIVE_FAILURE_LINE), [], "RSI blamed for a status stop")
        self._spin_for(0.5)
        self.assertEqual(self.stop_reason(), reason)

        log = LogCursor(proc_output)
        self.inject(clear)
        self.resume_and_activate(log)
        self.assertEqual(self.stop_reason(), STOP_REASON_NONE)
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)

    def test_drives_off_deactivates(self, proc_output):
        self.stop_on_status(
            proc_output,
            "drives_powered=0",
            STOP_REASON_DRIVES_NOT_POWERED,
            "drives not powered",
            "drives_powered=1",
        )

    def test_estop_deactivates_with_estop_reason(self, proc_output):
        self.stop_on_status(
            proc_output,
            "emergency_stop=1",
            STOP_REASON_EMERGENCY_STOP,
            "emergency stop",
            "emergency_stop=0",
        )

    def test_guard_stop_deactivates_with_guard_stop_reason(self, proc_output):
        self.stop_on_status(
            proc_output, "guard_stop=1", STOP_REASON_GUARD_STOP, "guard stop", "guard_stop=0"
        )

    def test_op_mode_change_deactivates(self, proc_output):
        self.stop_on_status(
            proc_output,
            f"operation_mode={OP_MODE_T1}",
            STOP_REASON_NOT_IN_EXT,
            "KRC not in EXT",
            "operation_mode=4",
        )


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
