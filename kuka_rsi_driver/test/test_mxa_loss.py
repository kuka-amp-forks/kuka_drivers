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

"""A silent mxA link errors the active component to UNCONFIGURED; unchanged flags do not."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    JSB,
    JTC,
    MXA_LOSS_LINE,
    MXA_RELEASED_LINE,
    RECEIVE_FAILURE_LINE,
    STOP_REASON_BROADCASTER,
    STOP_REASON_MXA_CONNECTION_LOST,
    STOP_REASON_NONE,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

# The driver's default bound; the fake ticks a telegram every 10 ms.
MXA_STATUS_TIMEOUT_MS = 60
# Long enough for the bound to expire many times over on an idle but healthy stream.
UNCHANGED_S = 2.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(respawn_simulator=True)


class TestMxaLoss(RsiFaultTestCase):
    def test_loss_while_active_errors_to_unconfigured(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)

        log = LogCursor(proc_output)
        self.inject("link_down")
        match = log.wait_for_match(MXA_LOSS_LINE, timeout=5)
        self.assertIsNotNone(match, "no mxA loss line in the driver log")
        bound, age = int(match.group(1)), int(match.group(2))
        self.assertEqual(bound, MXA_STATUS_TIMEOUT_MS)
        self.assertGreater(age, bound)
        self.assertTrue(log.wait_for(MXA_RELEASED_LINE, timeout=5), "client not released")
        self.wait_for_component_state(State.PRIMARY_STATE_UNCONFIGURED)
        self.assertEqual(self.controller_state(JTC), "inactive")
        self.assertEqual(self.controller_state(JSB), "inactive")
        self.assertEqual(self.controller_state(STOP_REASON_BROADCASTER), "inactive")
        self.assertEqual(log.matches(RECEIVE_FAILURE_LINE), [], "RSI blamed for the mxA loss")

        # A reconfigure reconnects; stop_reason reads the loss until activation succeeds.
        log = LogCursor(proc_output)
        self.inject("link_up")
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "reconfigure")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)
        self.switch(activate=[JSB, STOP_REASON_BROADCASTER])
        self.assertEqual(self.stop_reason(), STOP_REASON_MXA_CONNECTION_LOST)
        self.resume_and_activate(log)
        self.assertEqual(self.stop_reason(), STOP_REASON_NONE)
        self.switch(activate=[JTC])
        self.move_joint_and_wait()

    def test_unchanged_status_does_not_error(self, proc_output):
        # Runs after the loss test, with the component active and JTC active.
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)
        log = LogCursor(proc_output)
        self.inject("freeze_status")
        self.assertTrue(log.wait_for("mxA status fake: status frozen", timeout=5))
        self._spin_for(UNCHANGED_S)
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)
        self.assertIsNone(log.wait_for_match(MXA_LOSS_LINE, timeout=0.1), "idle stream errored")
        self.assertEqual(self.stop_reason(), STOP_REASON_NONE)

        self.inject("unfreeze_status")
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
