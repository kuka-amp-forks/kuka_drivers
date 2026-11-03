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

"""E-stop and guard stop latch stop_reason in any state; drives-off while inactive does not."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mxa_status_fake_support import OP_MODE_EXT  # noqa: E402
from rsi_fault_support import (  # noqa: E402
    ANY_STATE_STOP_LINE,
    STOP_REASON_EMERGENCY_STOP,
    STOP_REASON_GUARD_STOP,
    STOP_REASON_NONE,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(respawn_simulator=True)


class TestStatusLatchesWhileInactive(RsiFaultTestCase):
    def setUp(self):
        super().setUp()
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()

    def latch_while_inactive(self, proc_output, datagram, reason, name, clear):
        """Inject a status while inactive, check the any-state line names it, then clear it."""
        log = LogCursor(proc_output)
        self.inject(datagram)
        match = log.wait_for_match(ANY_STATE_STOP_LINE, timeout=5)
        self.assertIsNotNone(match, "no any-state stop line in the driver log")
        self.assertEqual(match.group(1), name)
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)
        self.assertEqual(self.stop_reason(), reason)

        # Latched state clears the same way every other reason does: the next
        # successful activation, not the clearing injection on its own.
        log = LogCursor(proc_output)
        self.inject(clear)
        self.assertEqual(self.stop_reason(), reason)
        self.resume_and_activate(log)
        self.assertEqual(self.stop_reason(), STOP_REASON_NONE)
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)

    def test_estop_latches_while_inactive(self, proc_output):
        self.latch_while_inactive(
            proc_output,
            "emergency_stop=1",
            STOP_REASON_EMERGENCY_STOP,
            "emergency stop",
            f"emergency_stop=0 operation_mode={OP_MODE_EXT} drives_powered=1 motion_possible=1",
        )

    def test_guard_stop_latches_while_inactive(self, proc_output):
        self.latch_while_inactive(
            proc_output,
            "guard_stop=1",
            STOP_REASON_GUARD_STOP,
            "guard stop",
            f"guard_stop=0 operation_mode={OP_MODE_EXT} drives_powered=1 motion_possible=1",
        )

    def test_drives_off_does_not_latch_while_inactive(self, proc_output):
        log = LogCursor(proc_output)
        self.inject("drives_powered=0")
        self._spin_for(1.0)
        self.assertIsNone(
            log.wait_for_match(ANY_STATE_STOP_LINE, timeout=0.1),
            "drives-off latched while inactive",
        )
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)
        self.assertEqual(self.stop_reason(), STOP_REASON_NONE)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
