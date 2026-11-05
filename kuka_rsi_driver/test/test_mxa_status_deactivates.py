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

"""Each mxA status that forbids motion deactivates the component and raises its slot."""

import os
import re
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mxa_status_fake_support import OP_MODE_T1  # noqa: E402
from rsi_fault_support import (  # noqa: E402
    INACTIVE_LATCH_LINE,
    JTC,
    RECEIVE_FAILURE_LINE,
    STATUS_STOP_LINE,
    UNATTRIBUTED_LINE,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

GUARD_STOP_INJECTED_LINE = re.compile(r"mxA status fake: guard_stop=1")
STOP_COMPLETE_LINE = re.compile(r"Stop complete")


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(respawn_simulator=True)


class TestMxaStatusDeactivates(RsiFaultTestCase):
    def stop_on_status(self, proc_output, datagram, name, clear, active, latched):
        """Stream, inject a status, check the stop names it and its slots, clear it, reactivate.

        active and latched name the slots after the stop; a stop with no slot active is the
        unattributed one, which raises rsi.
        """
        log = LogCursor(proc_output)
        self.start_streaming(log)

        log = LogCursor(proc_output)
        self.inject(datagram)
        match = log.wait_for_match(STATUS_STOP_LINE, timeout=5)
        self.assertIsNotNone(match, "no stop line in the driver log")
        self.assertEqual(match.group(1), name)
        self.assertTrue(datagram in match.group(2), "raw status not logged with the cause")
        self.assert_stopped(log, latched)
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.assertEqual(log.matches(RECEIVE_FAILURE_LINE), [], "RSI blamed for a status stop")
        self.assertEqual(
            len(log.matches(UNATTRIBUTED_LINE)), 1 if active == {"rsi"} else 0, "attribution"
        )
        self._spin_for(0.5)
        self.assert_active(active)
        self.assert_latched(latched)

        log = LogCursor(proc_output)
        self.inject(clear)
        self.resume_and_activate(log)
        self.assert_active(set())
        self.assert_latched(set())
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)

    def test_drives_off_deactivates(self, proc_output):
        # drives_powered is informational, so the stop is unattributed and raises rsi.
        self.stop_on_status(
            proc_output,
            "drives_powered=0",
            "drives not powered",
            "drives_powered=1",
            active={"rsi"},
            latched={"rsi"},
        )

    def test_estop_deactivates_with_estop_reason(self, proc_output):
        self.stop_on_status(
            proc_output,
            "emergency_stop=1",
            "emergency stop",
            "emergency_stop=0",
            active={"estop"},
            latched={"estop"},
        )

    def test_guard_stop_deactivates_with_guard_stop_reason(self, proc_output):
        self.stop_on_status(
            proc_output,
            "guard_stop=1",
            "guard stop",
            "guard_stop=0",
            active={"guard"},
            latched={"guard"},
        )

    def test_guard_stop_during_own_stop_is_not_latched(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)
        # A slow cancel keeps the stop pending long enough to inject into.
        self.inject("cancel_cycles=100")
        self.switch(deactivate=[JTC])

        log = LogCursor(proc_output)
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.inject("guard_stop=1")
        injected = log.wait_for_match(GUARD_STOP_INJECTED_LINE, timeout=5)
        self.assertIsNotNone(injected, "guard stop not delivered")
        complete = log.wait_for_match(STOP_COMPLETE_LINE, timeout=5)
        self.assertIsNotNone(complete, "stop never completed")
        self.assertLess(injected.start(), complete.start(), "stop completed before the guard stop")
        self._spin_for(0.5)
        self.assertEqual(log.matches(INACTIVE_LATCH_LINE), [], "own stop latched the guard stop")
        self.assert_active({"guard"})
        self.assert_latched(set())

        self.inject("guard_stop=0 cancel_cycles=12")
        self.assert_active(set())
        self.wait_for_simulator_respawn(log)

    def test_op_mode_change_deactivates(self, proc_output):
        # The key switch is its own release: mode is active at the stop and never latched.
        self.stop_on_status(
            proc_output,
            f"operation_mode={OP_MODE_T1}",
            "KRC not in EXT",
            "operation_mode=4",
            active={"mode"},
            latched=set(),
        )


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
