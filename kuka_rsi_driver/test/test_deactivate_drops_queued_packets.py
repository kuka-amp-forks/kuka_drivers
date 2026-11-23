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

"""A deactivate with no packet to answer drops the queued packets before the next activation."""

import os
import re
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    NO_STOP_PACKET_LINE,
    SIMULATOR_SILENCE_PACKETS,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
    hardware_params,
)

FAULT_CONTROLLER = "fault_injection_controller"
MUTE_MS = 1500
# Packets the simulator sends into the mute before it is silenced, at one per 40 ms cycle.
QUEUED_PACKETS = 4
MUTE_START_LINE = re.compile(r"Withholding RSI replies for (\d+) ms after IPOC (\d+)")
MUTE_STOP_LINE = re.compile(r"Stopping: RSI replies withheld")
DISCARDED_LINE = re.compile(r"Discarded (\d+) RSI packet\(s\) queued before the start request")
FIRST_PACKET_LINE = re.compile(
    r"Received position data from robot controller (\d+) ms after the start request, IPOC (\d+)"
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(
        inactive_controllers=[(FAULT_CONTROLLER, "fault_injection_controller.yaml")],
        extra_params=hardware_params(fault_injection="true"),
    )


class TestDeactivateDropsQueuedPackets(RsiFaultTestCase):
    def test_deactivate_drops_queued_packets(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)
        self.switch(activate=[FAULT_CONTROLLER])

        # Packets queue unread while the replies are withheld; then the simulator goes quiet, as
        # the cancelled RSI program would, before the mute ends in a stop with no packet to answer.
        log = LogCursor(proc_output)
        msg = DynamicInterfaceGroupValues()
        msg.interface_groups = ["gpio"]
        msg.interface_values = [InterfaceValue(interface_names=["rsi_mute_ms"], values=[MUTE_MS])]
        publisher = self.node.create_publisher(
            DynamicInterfaceGroupValues, f"/{FAULT_CONTROLLER}/commands", 10
        )
        for _ in range(3):
            publisher.publish(msg)
            self._spin_for(0.05)
        start = log.wait_for_match(MUTE_START_LINE, timeout=5)
        self.assertIsNotNone(start, "no mute start line")
        muted_after = int(start.group(2))
        self.assertTrue(
            self._spin_until(
                lambda: len([i for i in self.state_ipocs if i > muted_after]) >= QUEUED_PACKETS,
                timeout=5,
            ),
            "too few packets queued in the mute",
        )
        self.set_simulator_fault("fault_dropped_packet_count", SIMULATOR_SILENCE_PACKETS)
        self.assertIsNotNone(log.wait_for_match(MUTE_STOP_LINE, timeout=5), "no mute stop line")
        self.wait_for_component_state(State.PRIMARY_STATE_INACTIVE)
        # The first inhibitor sample after the stop can predate the latch.
        self.assertTrue(
            self._spin_until(lambda: self.latched() == {"rsi"}, timeout=2),
            "rsi not latched after the stop",
        )
        self.assertTrue(log.wait_for(NO_STOP_PACKET_LINE, timeout=5), "stop packet attempted")
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        queued = [i for i in self.state_ipocs if i > muted_after]

        log = LogCursor(proc_output)
        self.resume_and_activate(log)
        first = log.wait_for_match(FIRST_PACKET_LINE, timeout=5)
        self.assertIsNotNone(first, "no first-packet line")
        self.assertGreater(int(first.group(2)), max(queued))
        self.assertEqual(log.matches(DISCARDED_LINE), [], "a packet of the earlier run was queued")


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
