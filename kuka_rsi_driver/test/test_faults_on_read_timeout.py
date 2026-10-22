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

"""A packet later than read_timeout_ms faults the component; the late packet goes unanswered."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    RECEIVE_FAILURE_LINE,
    STOP_REASON_RSI_RECEIVE_FAILED,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

# Two simulator cycles; the late packet arrives after four.
READ_TIMEOUT_MS = 80
LATE_CYCLES = 4


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(read_timeout_ms=READ_TIMEOUT_MS)


class TestFaultsOnReadTimeout(RsiFaultTestCase):
    def test_faults_on_read_timeout(self, proc_output):
        log = LogCursor(proc_output)
        self.assertTrue(log.wait_for(f"RSI read timeout: {READ_TIMEOUT_MS} ms", timeout=10))
        self.start_streaming(log)

        log = LogCursor(proc_output)
        self.set_simulator_fault("fault_late_packet_cycles", LATE_CYCLES)
        self.assertTrue(
            log.wait_for(
                f"Late packet injected, deferring send ({LATE_CYCLES - 1} cycle(s) left)", 5
            )
        )
        match = log.wait_for_match(RECEIVE_FAILURE_LINE, timeout=5)
        self.assertIsNotNone(match, "no stop line in the driver log")
        self.assertEqual(int(match.group(1)), READ_TIMEOUT_MS)
        latched_ipoc = int(match.group(2))
        self.assert_stopped(log, STOP_REASON_RSI_RECEIVE_FAILED)

        # The delayed packet is the next IPOC; it arrives after the latch and gets no answer.
        self.wait_for_states_after(latched_ipoc, count=2)
        self.assertEqual(min(i for i in self.state_ipocs if i > latched_ipoc), latched_ipoc + 1)
        self.assertEqual(self.commands_after(latched_ipoc), [], "command sent after the latch")


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
