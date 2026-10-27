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

"""A stall on the driver's side, with the KRC still sending, is not an mxA loss."""

import os
import re
import signal
import sys
import time
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    EVENT_BROADCASTER,
    JSB,
    JTC,
    MXA_LOSS_LINE,
    STOP_REASON_BROADCASTER,
    STOP_REASON_NONE,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

# The driver's default bound.
MXA_STATUS_TIMEOUT_MS = 60
# Past the bound and below the SDK's 100 ms receive timeout, like a CFS quota freeze.
STALL_MS = 90
STALLS = 5
PROCESS_STOPS = 10
RECEIVE_RESUMED_LINE = re.compile(re.escape("mxA status fake: receive resumed"))


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(respawn_simulator=True)


class TestReceiveStallDoesNotError(RsiFaultTestCase):
    def prepare_inactive(self):
        """INACTIVE with read() running and the broadcasters the spawners started still active."""
        for name in (JSB, STOP_REASON_BROADCASTER, EVENT_BROADCASTER):
            self._wait_for_controller_active(name, timeout=30)
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()

    def stall_receive(self, log):
        """Hold the fake's receive thread past the bound, again and again."""
        for _ in range(STALLS):
            self.inject(f"stall_receive={STALL_MS}")
        self.assertTrue(
            self._spin_until(lambda: len(log.matches(RECEIVE_RESUMED_LINE)) == STALLS, timeout=5),
            "the receive thread did not resume",
        )

    def assert_no_loss(self, log, state, stop_reason):
        self.assertEqual(log.matches(MXA_LOSS_LINE), [], "a local stall read as an mxA loss")
        self.assertEqual(self.component_state(), state)
        self.assertEqual(self.stop_reason(), stop_reason)

    def test_process_stop_while_inactive(self, proc_output, proc_info):
        self.prepare_inactive()
        held = self.stop_reason()
        pid = proc_info["control_node"].pid

        # The whole process, fake included, freezes as under a CPU quota.
        log = LogCursor(proc_output)
        for _ in range(PROCESS_STOPS):
            os.kill(pid, signal.SIGSTOP)
            time.sleep(STALL_MS / 1000)
            os.kill(pid, signal.SIGCONT)
            self._spin_for(0.2)
        self.assert_no_loss(log, State.PRIMARY_STATE_INACTIVE, held)

    def test_receive_stall_while_active(self, proc_output):
        log = LogCursor(proc_output)
        self.prepare_inactive()
        self.start_streaming(log)

        log = LogCursor(proc_output)
        self.stall_receive(log)
        self.assert_no_loss(log, State.PRIMARY_STATE_ACTIVE, STOP_REASON_NONE)
        self.assertEqual(self.controller_state(JTC), "active")

        log = LogCursor(proc_output)
        self.switch(deactivate=[JTC])
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)

    def test_receive_stall_while_inactive(self, proc_output):
        self.prepare_inactive()
        held = self.stop_reason()

        log = LogCursor(proc_output)
        self.stall_receive(log)
        self.assert_no_loss(log, State.PRIMARY_STATE_INACTIVE, held)

        # A silent link after the stalls still errors within the bound.
        self.inject("link_down")
        match = log.wait_for_match(MXA_LOSS_LINE, timeout=5)
        self.assertIsNotNone(match, "no mxA loss line in the driver log")
        bound, age = int(match.group(1)), int(match.group(2))
        self.assertEqual(bound, MXA_STATUS_TIMEOUT_MS)
        self.assertGreater(age, bound)
        self.wait_for_component_state(State.PRIMARY_STATE_UNCONFIGURED)

        self.inject("link_up")
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "reconfigure")
        self.switch(activate=[JSB, STOP_REASON_BROADCASTER, EVENT_BROADCASTER])


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
