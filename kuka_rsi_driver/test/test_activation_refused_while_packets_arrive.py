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

"""Activation is refused while RSI packets keep arriving before the start request."""

import os
import re
import subprocess
import sys
import time
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    OP_MODE_EXT,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

STILL_ARRIVING_LINE = re.compile(
    r"Activation failed: RSI packets still arriving after dropping (\d+), IPOC (\d+) to (\d+)"
)
STARTING_LINE = "Starting joint position control at 4 ms"

# Sends RSI state packets to the driver's port without pause, as an RSI program that was never
# cancelled keeps doing; its own process, so the test's executor never opens a gap in the stream.
STREAM_SCRIPT = """
import socket
import numpy as np
from kuka_rsi_simulator.rsi_simulator import _default_rsi_xml_config, create_rsi_xml_rob
config = _default_rsi_xml_config()
pos = np.zeros(12)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ipoc = 1
while True:
    sock.sendto(create_rsi_xml_rob(pos, pos, pos, 0, ipoc, config), ("127.0.0.1", 59152))
    ipoc += 1
"""


class _StaleStream:
    def __enter__(self):
        self._proc = subprocess.Popen([sys.executable, "-c", STREAM_SCRIPT])
        time.sleep(1.0)
        return self

    def __exit__(self, *exc):
        self._proc.terminate()
        self._proc.wait(timeout=10)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(respawn_simulator=True)


class TestActivationRefusedWhilePacketsArrive(RsiFaultTestCase):
    def test_activation_refused_while_packets_arrive(self, proc_output):
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()
        self.inject(f"operation_mode={OP_MODE_EXT} drives_powered=1 motion_possible=1")

        log = LogCursor(proc_output)
        with _StaleStream():
            self.assertFalse(
                self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activated on a stream"
            )
        match = log.wait_for_match(STILL_ARRIVING_LINE, timeout=1)
        self.assertIsNotNone(match, "not refused for packets still arriving")
        self.assertFalse(log.wait_for(STARTING_LINE, timeout=0.1), "RSI start requested")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)
        self.assert_latched(set())

        # Once the stream stops, the leftovers are dropped and activation succeeds.
        log = LogCursor(proc_output)
        self.resume_and_activate(log)
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)

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
