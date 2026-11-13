# Copyright 2026 KUKA Hungaria Kft.
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

"""Packets queued while INACTIVE are dropped; activation takes its first packet after the start."""

import os
import re
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    JTC,
    RECEIVE_FAILURE_LINE,
    SIMULATOR_SILENCE_PACKETS,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

# 200 ms of silence at the simulator's 40 ms cycle, twice the read timeout.
DROPPED_PACKETS = 5
# Packets the simulator sends after the drop, unread while the component is INACTIVE.
QUEUED_PACKETS = 3

FIRST_PACKET_LINE = re.compile(
    r"Received position data from robot controller (\d+) ms after the start request, IPOC (\d+)"
)
DISCARDED_LINE = re.compile(
    r"Discarded (\d+) RSI packet\(s\) queued before the start request, IPOC (\d+) to (\d+)"
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description()


class TestReactivationDiscardsQueuedPackets(RsiFaultTestCase):
    def test_reactivation_discards_queued_packets(self, proc_output, proc_info):
        log = LogCursor(proc_output)
        self.start_streaming(log)

        log = LogCursor(proc_output)
        self.set_simulator_fault("fault_dropped_packet_count", DROPPED_PACKETS)
        match = log.wait_for_match(RECEIVE_FAILURE_LINE, timeout=5)
        self.assertIsNotNone(match, "no stop line in the driver log")
        latched_ipoc = int(match.group(2))
        self.assert_stopped(log, {"rsi"})
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")

        # Let packets queue unread, then silence the simulator as a cancelled RSI program would.
        self.wait_for_states_after(latched_ipoc, count=QUEUED_PACKETS)
        self.set_simulator_fault("fault_dropped_packet_count", SIMULATOR_SILENCE_PACKETS)
        self._spin_for(0.5)
        queued = [i for i in self.state_ipocs if i > latched_ipoc]
        self.assertGreaterEqual(len(queued), QUEUED_PACKETS, self.state_ipocs)

        log = LogCursor(proc_output)
        self.resume_and_activate(log)
        first = log.wait_for_match(FIRST_PACKET_LINE, timeout=5)
        self.assertIsNotNone(first, "no first-packet line")
        self.assertGreater(
            int(first.group(2)), max(queued), "activation took a packet queued before the start"
        )
        discarded = log.wait_for_match(DISCARDED_LINE, timeout=1)
        self.assertIsNotNone(discarded, "no discard line")
        self.assertEqual(
            (int(discarded.group(2)), int(discarded.group(3))), (min(queued), max(queued))
        )
        self.assert_latched(set())

        self.switch(activate=[JTC])
        self.move_joint_and_wait()
        self.switch(deactivate=[JTC])
        self.deactivate_and_wait_for_simulator(proc_info)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
