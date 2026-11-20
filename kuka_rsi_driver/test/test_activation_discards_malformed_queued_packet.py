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

"""A malformed packet queued while INACTIVE fails activation cleanly instead of throwing."""

import os
import re
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    OP_MODE_EXT,
    RECEIVE_FAILURE_LINE,
    SIMULATOR_SILENCE_PACKETS,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

# Sent while INACTIVE, ahead of the activation request: queued for DiscardQueuedPackets to read.
QUEUED_MALFORMED = 1
DROPPED_PACKETS = 5

MALFORMED_INJECTED_LINE = "Malformed packet injected"
DISCARD_PARSE_FAILURE_LINE = re.compile(
    r"Activation failed: parsing a queued RSI packet failed \(.+\), (\d+) bytes: (\S.*)"
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description()


class TestActivationDiscardsMalformedQueuedPacket(RsiFaultTestCase):
    def test_activation_discards_malformed_queued_packet(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)

        log = LogCursor(proc_output)
        self.set_simulator_fault("fault_dropped_packet_count", DROPPED_PACKETS)
        match = log.wait_for_match(RECEIVE_FAILURE_LINE, timeout=5)
        self.assertIsNotNone(match, "no stop line in the driver log")
        self.assert_stopped(log, {"rsi"})
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")

        # Queue a malformed packet while INACTIVE, then silence the simulator behind it.
        injected = LogCursor(proc_output)
        self.set_simulator_fault("fault_malformed_packet_count", QUEUED_MALFORMED)
        self.assertTrue(
            injected.wait_for(MALFORMED_INJECTED_LINE, timeout=5),
            "malformed packet never injected",
        )
        self.set_simulator_fault("fault_dropped_packet_count", SIMULATOR_SILENCE_PACKETS)

        log = LogCursor(proc_output)
        self.inject(f"operation_mode={OP_MODE_EXT} drives_powered=1 motion_possible=1")
        ok = self.set_component_state(State.PRIMARY_STATE_ACTIVE)
        self.assertFalse(ok, "activation unexpectedly succeeded")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)

        match = log.wait_for_match(DISCARD_PARSE_FAILURE_LINE, timeout=5)
        self.assertIsNotNone(
            match,
            "no discard parse-failure line; activation did not fail cleanly on the malformed "
            "queued packet",
        )


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
