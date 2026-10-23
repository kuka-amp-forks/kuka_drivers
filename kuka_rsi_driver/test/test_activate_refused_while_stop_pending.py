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

"""Activation is refused while the previous stop is pending, and the pending stop is bounded."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mxa_status_fake_support import (  # noqa: E402
    COMPONENT,
    OP_MODE_EXT,
    STOP_COMPLETE_DRIVES_ON_LINE,
    STOP_COMPLETE_LINE,
    STOP_PENDING_FAILURE,
    LogCursor,
    MxaStatusFakeTestCase,
    fake_launch_description,
    hardware_params,
)

STOP_PENDING_TIMEOUT_MS = 2000
# Serve ticks of 10 ms the fake's cancel takes: a window to activate into, under the bound.
CANCEL_CYCLES = 60
MARGIN_S = 3.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fake_launch_description(
        extra_params=hardware_params(stop_pending_timeout_ms=STOP_PENDING_TIMEOUT_MS),
        respawn_simulator=True,
    )


class TestActivateRefusedWhileStopPending(MxaStatusFakeTestCase):
    def setUp(self):
        super().setUp()
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()
        self.inject(f"unfreeze_status drives_stuck=0 cancel_cycles={CANCEL_CYCLES}")

    def deactivate_then_activate_at_once(self, log):
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertFalse(self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activated")
        self.assertTrue(log.wait_for(STOP_PENDING_FAILURE, timeout=5), "cause not named")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)

    def activate_after_stop(self, log):
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activate refused")
        self.assertTrue(log.wait_for(f"Successful 'activate' of hardware '{COMPONENT}'", 30))

    def finish(self, log):
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for(STOP_COMPLETE_LINE, timeout=5), "stop never completed")

    def test_activate_refused_while_stop_pending(self, proc_output):
        log = LogCursor(proc_output)
        self.activate_and_stream(log)

        log = LogCursor(proc_output)
        self.deactivate_then_activate_at_once(log)
        self.assertTrue(log.wait_for("RSI program cancel not confirmed", timeout=1))
        self.assertTrue(log.wait_for(STOP_COMPLETE_LINE, timeout=5), "stop never completed")

        log = LogCursor(proc_output)
        self.activate_after_stop(log)
        self.finish(log)

    def test_stop_pending_holds_while_cancel_unconfirmed(self, proc_output):
        log = LogCursor(proc_output)
        self.activate_and_stream(log)
        self.inject("freeze_status")

        log = LogCursor(proc_output)
        self.deactivate_then_activate_at_once(log)
        self.assertTrue(
            log.wait_for(
                f"RSI program cancel not confirmed within {STOP_PENDING_TIMEOUT_MS} ms",
                timeout=STOP_PENDING_TIMEOUT_MS / 1000 + MARGIN_S,
            ),
            "bound expired silently",
        )
        self.assertFalse(log.wait_for("Stop complete", timeout=0.1), "stop settled unconfirmed")

        log = LogCursor(proc_output)
        self.assertFalse(self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activated")
        self.assertTrue(
            log.wait_for(f"{STOP_PENDING_FAILURE}, RSI program cancel not confirmed", 5)
        )

        log = LogCursor(proc_output)
        self.inject("unfreeze_status")
        self.assertTrue(log.wait_for(STOP_COMPLETE_LINE, timeout=5), "stop never completed")
        self.activate_after_stop(log)
        self.finish(log)

    def test_stop_pending_settles_with_drives_stuck_on(self, proc_output):
        log = LogCursor(proc_output)
        self.activate_and_stream(log)
        self.inject("drives_stuck=1")

        log = LogCursor(proc_output)
        self.deactivate_then_activate_at_once(log)
        self.assertTrue(
            log.wait_for(
                f"{STOP_COMPLETE_DRIVES_ON_LINE} after {STOP_PENDING_TIMEOUT_MS} ms",
                timeout=STOP_PENDING_TIMEOUT_MS / 1000 + MARGIN_S,
            ),
            "stop never settled with the drives on",
        )
        self.assertFalse(log.wait_for(STOP_COMPLETE_LINE, timeout=0.1), "drives reported off")

        log = LogCursor(proc_output)
        self.activate_after_stop(log)
        self.assertFalse(log.wait_for("Turning on drives", timeout=0.1), "drives were on")
        self.inject(f"drives_stuck=0 operation_mode={OP_MODE_EXT}")
        self.finish(log)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
