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

"""The first RSI packet and command are logged verbatim, and so is a packet that fails to parse."""

import os
import re
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    SIMULATOR_COMMAND_TOPIC,
    SIMULATOR_SILENCE_PACKETS,
    SIMULATOR_STATE_TOPIC,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

# kuka_rsi_simulator's MALFORMED_RSI_PACKET.
MALFORMED_PACKET = '<Rob TYPE="KUKA"><Broken/></Rob>'
FIRST_PACKET_LINE = re.compile(r"First RSI packet after the start request \((\d+) bytes\): (\S.*)")
FIRST_COMMAND_LINE = re.compile(r"First RSI command \((\d+) bytes\): (\S.*)")
PARSE_FAILURE_LINE = re.compile(r"Parsing the RSI packet failed \(.+\), (\d+) bytes: (\S.*)")


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description()


class TestLogsRsiXml(RsiFaultTestCase):
    def setUp(self):
        super().setUp()
        self.raw_states = []
        self.raw_commands = []
        self.node.create_subscription(
            String, SIMULATOR_STATE_TOPIC, lambda msg: self.raw_states.append(msg.data), 100
        )
        self.node.create_subscription(
            String, SIMULATOR_COMMAND_TOPIC, lambda msg: self.raw_commands.append(msg.data), 100
        )

    def assert_logged_verbatim(self, match, published):
        self.assertIsNotNone(match, "not logged")
        xml = match.group(2).rstrip()
        self.assertEqual(int(match.group(1)), len(xml.encode()))
        self.assertTrue(
            self._spin_until(lambda: xml in published, timeout=5), f"{xml} not in {published}"
        )

    def test_logs_rsi_xml(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)
        self.assert_logged_verbatim(log.wait_for_match(FIRST_PACKET_LINE, 5), self.raw_states)
        self.assert_logged_verbatim(log.wait_for_match(FIRST_COMMAND_LINE, 5), self.raw_commands)

        log = LogCursor(proc_output)
        self.set_simulator_fault("fault_malformed_packet_count", 1)
        match = log.wait_for_match(PARSE_FAILURE_LINE, timeout=5)
        self.assertIsNotNone(match, "parse failure not logged")
        self.assertEqual(match.group(2).rstrip(), MALFORMED_PACKET)
        self.assertEqual(int(match.group(1)), len(MALFORMED_PACKET))
        self.assert_stopped(log, {"rsi"})
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")

        # The stop leaves nothing behind: the next activation streams and logs its own first XML.
        self.set_simulator_fault("fault_dropped_packet_count", SIMULATOR_SILENCE_PACKETS)
        log = LogCursor(proc_output)
        self.resume_and_activate(log)
        self.assertIsNotNone(log.wait_for_match(FIRST_PACKET_LINE, 5), "no first packet logged")
        ipoc = max(self.state_ipocs)
        self.assertTrue(
            self._spin_until(lambda: len(self.commands_after(ipoc)) >= 3, timeout=5),
            f"no RSI commands after IPOC {ipoc}",
        )


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
