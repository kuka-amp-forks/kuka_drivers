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

"""The six inhibitor slots carry every stop cause under the latch rule; stop_reason is gone."""

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
    INACTIVE_LATCH_LINE,
    INHIBITORS,
    JTC,
    OPERATOR_ACTION,
    SLOT_INTERFACES,
    STATUS_STOP_LINE,
    UNATTRIBUTED_LINE,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

# Ticks (10 ms) the fake takes to confirm a cancel, long enough to inject into the pending stop.
SLOW_CANCEL_CYCLES = 100


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(respawn_simulator=True)


class TestInhibitorSlots(RsiFaultTestCase):
    def setUp(self):
        super().setUp()
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()
        # The fake starts with the mode unknown, which inhibitor/mode reads as not EXT.
        self.inject(f"operation_mode={OP_MODE_EXT}")

    def reactivate_and_park(self, proc_output, log):
        """Let the pending stop finish, reactivate with the flags clear, then deactivate."""
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)
        log = LogCursor(proc_output)
        self.resume_and_activate(log)
        self.assert_latched(set())
        self.assert_active(set())
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.assert_latched(set())
        self.wait_for_simulator_respawn(log)

    def test_estop_while_inactive_latches(self, proc_output):
        self.assert_latched(set())
        log = LogCursor(proc_output)
        self.inject("emergency_stop=1")
        match = log.wait_for_match(INACTIVE_LATCH_LINE, timeout=5)
        self.assertIsNotNone(match, "no inactive latch line")
        self.assertEqual(match.groups(), ("1", "0", "1", "0"))
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)
        self.assert_active({"estop"})
        self.assert_latched({"estop"})

        # Held through a refused activation; the release clears it.
        self.assertFalse(self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activated")
        self.assertTrue(log.wait_for("Activation failed: Emergency stop is active", timeout=5))
        self.assert_latched({"estop"})
        self.inject("emergency_stop=0")
        self.assert_active(set())
        self.assert_latched(set())

    def test_exports_inhibitor_slots(self, proc_output):
        result = self._call(
            ListHardwareInterfaces,
            "/controller_manager/list_hardware_interfaces",
            ListHardwareInterfaces.Request(),
            timeout=30,
        )
        states = [i.name for i in result.state_interfaces]
        for name in INHIBITORS:
            for interface in SLOT_INTERFACES:
                full = f"inhibitor/{name}/{interface}"
                self.assertTrue(full in states, f"{full} not exported: {states}")
        self.assertFalse(
            [n for n in states if n.endswith("/stop_reason")], f"stop_reason exported: {states}"
        )

        slots = self.inhibitors()
        for name in INHIBITORS:
            self.assertEqual(slots[name]["operator_action"], OPERATOR_ACTION[name], name)
            self.assertEqual(slots[name]["latched"], 0.0, name)

        # active is the live signal; while INACTIVE neither the key switch nor a brake-test
        # request latches.
        self.inject(f"operation_mode={OP_MODE_T1} brake_test_requested=1")
        self.assert_active({"mode", "brake_test"})
        self.assert_latched(set())
        self.inject(f"operation_mode={OP_MODE_EXT} brake_test_requested=0")
        self.assert_active(set())
        self.assert_latched(set())

    def test_mode_exit_reports_mode_and_guard(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)

        # The KRC raises a safety stop when the key leaves EXT, so both arrive together.
        log = LogCursor(proc_output)
        self.inject(f"guard_stop=1 operation_mode={OP_MODE_T1}")
        self.assertIsNotNone(log.wait_for_match(STATUS_STOP_LINE, timeout=5), "no stop line")
        self.assert_stopped(log, {"guard"})
        self.assert_active({"guard", "mode"})
        self.assertEqual(log.matches(UNATTRIBUTED_LINE), [], "attributed, yet rsi raised")

        # Each release clears its own slot; mode was never latched.
        self.inject("guard_stop=0")
        self.assert_active({"mode"})
        self.assert_latched(set())
        self.inject(f"operation_mode={OP_MODE_EXT}")
        self.assert_active(set())
        self.reactivate_and_park(proc_output, log)

    def test_own_deactivate_latches_no_slot(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)
        self.inject(f"cancel_cycles={SLOW_CANCEL_CYCLES}")
        self.switch(deactivate=[JTC])

        # What the r930 reports after our own drives-off, while the stop is still pending.
        log = LogCursor(proc_output)
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.inject("motion_possible=0 guard_stop=1")
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.assert_active({"guard"})
        self.assert_latched(set())
        self.assertEqual(log.matches(UNATTRIBUTED_LINE), [], "own stop raised rsi")
        self.assertEqual(log.matches(INACTIVE_LATCH_LINE), [], "own stop latched the guard")

        self.inject("guard_stop=0 motion_possible=1 cancel_cycles=12")
        self.assert_active(set())
        self.assert_latched(set())
        self.wait_for_simulator_respawn(log)

    def test_simultaneous_causes_read_as_two_slots(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)

        log = LogCursor(proc_output)
        self.inject("emergency_stop=1 guard_stop=1")
        self.assertIsNotNone(log.wait_for_match(STATUS_STOP_LINE, timeout=5), "no stop line")
        self.assert_stopped(log, {"estop", "guard"})
        self.assert_active({"estop", "guard"})
        self.assertEqual(log.matches(UNATTRIBUTED_LINE), [], "attributed, yet rsi raised")

        # Each release clears its own latch.
        self.inject("guard_stop=0")
        self.assert_active({"estop"})
        self.assert_latched({"estop"})
        self.inject("emergency_stop=0")
        self.assert_active(set())
        self.assert_latched(set())
        self.reactivate_and_park(proc_output, log)

    def test_unattributed_stop_raises_rsi(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)

        # motion_possible is informational: a stop with no slot active means the command link
        # to the controller is lost.
        log = LogCursor(proc_output)
        self.inject("motion_possible=0")
        self.assertIsNotNone(log.wait_for_match(STATUS_STOP_LINE, timeout=5), "no stop line")
        match = log.wait_for_match(UNATTRIBUTED_LINE, timeout=5)
        self.assertIsNotNone(match, "no unattributed line")
        self.assertEqual(match.group(1), "motion not possible")
        self.assert_stopped(log, {"rsi"})
        self.assert_active({"rsi"})

        # rsi shows no release: it holds until the activation succeeds.
        self.inject("motion_possible=1")
        self.assert_active({"rsi"})
        self.assert_latched({"rsi"})
        self.reactivate_and_park(proc_output, log)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
