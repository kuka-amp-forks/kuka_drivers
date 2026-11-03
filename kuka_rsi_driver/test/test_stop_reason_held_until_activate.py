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

"""stop_reason holds the cause through INACTIVE and a refused activation; success resets it."""

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
    JTC,
    RECEIVE_FAILURE_LINE,
    STOP_REASON_NONE,
    STOP_REASON_RSI_RECEIVE_FAILED,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

# Silence until the test resumes the simulator, so nothing queues while INACTIVE.
SILENCE_PACKETS = 100000


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description()


class TestStopReasonHeldUntilActivate(RsiFaultTestCase):
    def test_stop_reason_held_until_activate(self, proc_output, proc_info):
        log = LogCursor(proc_output)
        self.start_streaming(log)

        log = LogCursor(proc_output)
        self.set_simulator_fault("fault_dropped_packet_count", SILENCE_PACKETS)
        self.assertIsNotNone(log.wait_for_match(RECEIVE_FAILURE_LINE, timeout=5), "no stop line")
        self.assert_stopped(log, STOP_REASON_RSI_RECEIVE_FAILED)
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self._spin_for(1.0)
        self.assertEqual(self.stop_reason(), STOP_REASON_RSI_RECEIVE_FAILED)

        # A refused activation keeps the cause.
        log = LogCursor(proc_output)
        self.inject(f"operation_mode={OP_MODE_T1}")
        self.assertFalse(self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activated in T1")
        self.assertTrue(log.wait_for("Activation failed: KRC not in EXT", timeout=5))
        self.assertEqual(self.stop_reason(), STOP_REASON_RSI_RECEIVE_FAILED)

        # A successful one resets it, and it stays 0 through a normal deactivate.
        log = LogCursor(proc_output)
        self.resume_and_activate(log)
        self.assertEqual(self.stop_reason(), STOP_REASON_NONE)
        self.switch(activate=[JTC])
        self.move_joint_and_wait()
        self.switch(deactivate=[JTC])
        self.deactivate_and_wait_for_simulator(proc_info)
        self.assertEqual(self.stop_reason(), STOP_REASON_NONE)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
