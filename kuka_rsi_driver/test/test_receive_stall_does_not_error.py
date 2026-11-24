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

"""A stall of the driver's whole process, with the KRC still sending, is not an mxA loss."""

import os
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
    MXA_LOSS_LINE,
    INHIBITOR_BROADCASTER,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

# Past the driver's bound and below the SDK's 100 ms receive timeout, like a CFS quota freeze.
STALL_MS = 90
PROCESS_STOPS = 10


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(respawn_simulator=True)


class TestReceiveStallDoesNotError(RsiFaultTestCase):
    def prepare_inactive(self):
        """INACTIVE with read() running and the broadcasters active, also after an mxA loss."""
        if self.component_state() == State.PRIMARY_STATE_UNCONFIGURED:
            self.inject("link_up")
            self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "reconfigure")
            self.switch(activate=[JSB, INHIBITOR_BROADCASTER, EVENT_BROADCASTER])
        for name in (JSB, INHIBITOR_BROADCASTER, EVENT_BROADCASTER):
            self._wait_for_controller_active(name, timeout=30)
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()

    def assert_still(self, log, state):
        """No loss line, and no error either: a held mxA-loss cause logs no second line."""
        self.assertEqual(log.matches(MXA_LOSS_LINE), [], "a local stall read as an mxA loss")
        self.assertEqual(self.component_state(), state, "a local stall errored the component")

    def test_process_stop_while_inactive(self, proc_output, proc_info):
        self.prepare_inactive()
        held = self.latched()
        pid = proc_info["control_node"].pid

        # The whole process, fake included, freezes as under a CPU quota.
        log = LogCursor(proc_output)
        for _ in range(PROCESS_STOPS):
            os.kill(pid, signal.SIGSTOP)
            time.sleep(STALL_MS / 1000)
            os.kill(pid, signal.SIGCONT)
            self._spin_for(0.2)
            self.assert_still(log, State.PRIMARY_STATE_INACTIVE)
        self.assertEqual(self.latched(), held)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
