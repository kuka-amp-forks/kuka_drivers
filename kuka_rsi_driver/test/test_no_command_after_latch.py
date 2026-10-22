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

"""After the latch the driver answers no RSI packet, not even with the stop packet."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    NO_STOP_PACKET_LINE,
    RECEIVE_FAILURE_LINE,
    STOP_REASON_RSI_RECEIVE_FAILED,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

DROPPED_PACKETS = 5
# States the simulator sends after the gap before the verdict; each repeats the unanswered IPOC.
UNANSWERED_STATES = 3


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description()


class TestNoCommandAfterLatch(RsiFaultTestCase):
    def test_no_command_after_latch(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)

        log = LogCursor(proc_output)
        self.set_simulator_fault("fault_dropped_packet_count", DROPPED_PACKETS)
        match = log.wait_for_match(RECEIVE_FAILURE_LINE, timeout=5)
        self.assertIsNotNone(match, "no stop line in the driver log")
        latched_ipoc = int(match.group(2))
        self.assert_stopped(log, STOP_REASON_RSI_RECEIVE_FAILED)
        self.assertTrue(log.wait_for(NO_STOP_PACKET_LINE, timeout=5), "stop packet attempted")

        self.wait_for_states_after(latched_ipoc, count=UNANSWERED_STATES)
        self.assertTrue(log.wait_for("Socket timed out", timeout=5), "simulator got an answer")
        self.assertEqual(self.commands_after(latched_ipoc), [], "command sent after the latch")
        commands = self.logged_commands(log)
        self.assertEqual(commands[-1], (latched_ipoc, False), commands)
        self.assertTrue(all(i <= latched_ipoc and not stop for i, stop in commands), commands)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
