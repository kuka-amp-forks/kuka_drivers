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

"""With no RSI packets, activation fails within its bound and leaves the RSI program cancelled."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mxa_status_fake_support import (  # noqa: E402
    FAKE_CANCEL_REQUESTED,
    FIRST_PACKET_FAILURE,
    OP_MODE_EXT,
    RECEIVED_FIRST_PACKET,
    STOP_COMPLETE_LINE,
    LogCursor,
    MxaStatusFakeTestCase,
    fake_launch_description,
    hardware_params,
)

FIRST_PACKET_TIMEOUT_MS = 500
# Covers the service round trip and the controller manager's transition thread on a slow
# runner, while staying under the old fixed 5 s wait.
MARGIN_S = 3.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fake_launch_description(
        extra_params=hardware_params(first_packet_timeout_ms=FIRST_PACKET_TIMEOUT_MS),
        simulator=False,
    )


class TestActivateBoundedWithoutFirstPacket(MxaStatusFakeTestCase):
    def setUp(self):
        super().setUp()
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()
        self.inject(f"operation_mode={OP_MODE_EXT} drives_powered=1 motion_possible=1")

    def activate_without_packets(self, log):
        ok, seconds = self.timed_set_component_state(State.PRIMARY_STATE_ACTIVE)
        self.assertFalse(ok, "activated with no RSI packet")
        self.assertTrue(
            log.wait_for(f"{FIRST_PACKET_FAILURE} {FIRST_PACKET_TIMEOUT_MS} ms", timeout=5)
        )
        self.assertFalse(log.wait_for(RECEIVED_FIRST_PACKET, timeout=0.1), "logged a packet")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)
        return seconds

    def test_activate_bounded_without_first_packet(self, proc_output):
        log = LogCursor(proc_output)
        seconds = self.activate_without_packets(log)
        bound_s = FIRST_PACKET_TIMEOUT_MS / 1000
        self.assertGreaterEqual(seconds, bound_s, f"gave up after {seconds:.2f} s")
        self.assertLess(seconds, bound_s + MARGIN_S, f"first-packet wait took {seconds:.2f} s")
        self.assertTrue(log.wait_for(STOP_COMPLETE_LINE, timeout=5), "stop never completed")

    def test_failed_first_read_cancels_rsi(self, proc_output):
        log = LogCursor(proc_output)
        self.activate_without_packets(log)
        self.assertTrue(log.wait_for(FAKE_CANCEL_REQUESTED, timeout=5), "no cancel requested")
        self.assertFalse(log.wait_for("Control signal not initialized", 0.1), "write attempted")

        report = self.fake_report(proc_output)
        self.assertEqual(report.group(1), "0", "RSI context left running")
        self.assertTrue(log.wait_for(STOP_COMPLETE_LINE, timeout=5), "stop never completed")
        report = self.fake_report(proc_output)
        self.assertEqual(report.group(3), "1", "cancel not done")
        self.assertEqual(report.group(4), "0", "drives left on")


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
