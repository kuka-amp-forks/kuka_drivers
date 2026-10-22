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

"""Dropped RSI packets take the component INACTIVE with JTC inactive and JSB still active."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    RECEIVE_FAILURE_LINE,
    SIMULATOR_READ_TIMEOUT_MS,
    STOP_REASON_RSI_RECEIVE_FAILED,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

# 200 ms of silence at the simulator's 40 ms cycle, twice the read timeout.
DROPPED_PACKETS = 5


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description()


class TestDeactivatesOnReceiveFailure(RsiFaultTestCase):
    def test_deactivates_on_receive_failure(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)

        log = LogCursor(proc_output)
        self.set_simulator_fault("fault_dropped_packet_count", DROPPED_PACKETS)
        match = log.wait_for_match(RECEIVE_FAILURE_LINE, timeout=5)
        self.assertIsNotNone(match, "no stop line in the driver log")
        self.assertEqual(int(match.group(1)), SIMULATOR_READ_TIMEOUT_MS)
        latched_ipoc = int(match.group(2))

        # The IPOC keeps advancing while packets are dropped, plus one for the last good
        # cycle's increment before the drop, so the simulator resumes exactly that far past
        # the latch.
        self.wait_for_states_after(latched_ipoc, count=1)
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)
        resumed_ipoc = min(i for i in self.state_ipocs if i > latched_ipoc)
        self.assertEqual(resumed_ipoc - latched_ipoc, DROPPED_PACKETS + 1)
        self.assert_stopped(log, STOP_REASON_RSI_RECEIVE_FAILED)
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
