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

"""Tests for the kuka_rsi_simulator fault-injection parameters.

Each test drives an RSISimulator node directly against a tiny fake RSI
client (a UDP echo peer) and observes the simulator's own rsi/state topic,
so none of this depends on kuka_rsi_driver's reaction to a fault.
"""

import itertools
import socket
import threading
import time
import unittest
import xml.etree.ElementTree as ET

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from kuka_rsi_simulator.rsi_simulator import RSISimulator

CYCLE_TIME = RSISimulator.cycle_time


class _FakeRsiClient:
    """A minimal RSI peer: echoes the IPOC it receives, applies no correction."""

    def __init__(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("127.0.0.1", 0))
        self._socket.settimeout(0.5)
        self.port = self._socket.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                data, addr = self._socket.recvfrom(2048)
            except socket.timeout:
                continue
            ipoc = ET.fromstring(data).find("IPOC").text
            reply = (
                b'<Sen><AK A1="0" A2="0" A3="0" A4="0" A5="0" A6="0"/>'
                b"<IPOC>" + ipoc.encode("utf-8") + b"</IPOC><Stop>0</Stop></Sen>"
            )
            self._socket.sendto(reply, addr)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
        self._socket.close()


class FaultInjectionTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.fake_client = _FakeRsiClient()
        self.node = RSISimulator("fault_injection_test")
        # The node reads its destination at construction time; point it at the
        # fake peer directly rather than via a parameter (no callback for this one).
        self.node.rsi_ip_address_ = "127.0.0.1"
        self.node.rsi_port_address_ = self.fake_client.port
        self.states = []
        self.node.create_subscription(
            String, f"{self.node.node_name_}/rsi/state", self._on_state, 10
        )

    def tearDown(self):
        self.node.destroy_node()
        self.fake_client.close()

    def _on_state(self, msg):
        root = ET.fromstring(msg.data)
        ipoc = int(root.find("IPOC").text)
        self.states.append((time.monotonic(), ipoc))

    def _spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _set_fault(self, name, value):
        result = self.node.set_parameters([Parameter(name, value=value)])[0]
        self.assertTrue(result.successful, result.reason)

    def test_default_behavior_is_unchanged(self):
        self._spin_for(6 * CYCLE_TIME)
        self.assertGreaterEqual(len(self.states), 4)
        ipocs = [ipoc for _, ipoc in self.states]
        self.assertEqual(ipocs, list(range(ipocs[0], ipocs[0] + len(ipocs))))
        gaps = [b - a for (a, _), (b, _) in zip(self.states, self.states[1:])]
        for gap in gaps:
            self.assertLess(gap, 2 * CYCLE_TIME)

    def test_simulator_injects_late_packet(self):
        self._spin_for(3 * CYCLE_TIME)
        self._set_fault("fault_late_packet_cycles", 3)
        before = len(self.states)
        self._spin_for(8 * CYCLE_TIME)
        self.assertGreater(len(self.states), before)

        gaps = [b - a for (a, _), (b, _) in zip(self.states, self.states[1:])]
        self.assertTrue(
            any(gap >= 2.5 * CYCLE_TIME for gap in gaps),
            f"no delayed gap observed, gaps={gaps}",
        )
        # The late packet is delayed, not skipped: IPOCs stay contiguous.
        ipocs = [ipoc for _, ipoc in self.states]
        self.assertEqual(ipocs, list(range(ipocs[0], ipocs[0] + len(ipocs))))

    def test_simulator_injects_dropped_packet(self):
        self._spin_for(3 * CYCLE_TIME)
        self._set_fault("fault_dropped_packet_count", 2)
        before = len(self.states)
        self._spin_for(8 * CYCLE_TIME)
        self.assertGreater(len(self.states), before)

        gaps = [b - a for (a, _), (b, _) in zip(self.states, self.states[1:])]
        self.assertTrue(
            any(gap >= 2.5 * CYCLE_TIME for gap in gaps),
            f"no dropped-cycle gap observed, gaps={gaps}",
        )
        # The dropped cycles are skipped, not replayed: IPOCs stay contiguous.
        ipocs = [ipoc for _, ipoc in self.states]
        self.assertEqual(ipocs, list(range(ipocs[0], ipocs[0] + len(ipocs))))

    def test_simulator_injects_repeated_ipoc(self):
        self._spin_for(3 * CYCLE_TIME)
        self._set_fault("fault_repeated_ipoc_count", 3)
        self._spin_for(8 * CYCLE_TIME)

        ipocs = [ipoc for _, ipoc in self.states]
        run_lengths = [len(list(group)) for _, group in itertools.groupby(ipocs)]
        # The armed IPOC is sent once normally, then repeated 3 more times.
        self.assertTrue(4 in run_lengths, f"no run of 4 identical IPOCs, ipocs={ipocs}")
        self.assertTrue(all(length == 1 for length in run_lengths if length != 4))


if __name__ == "__main__":
    unittest.main()
