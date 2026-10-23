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
import unittest
import xml.etree.ElementTree as ET

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from kuka_rsi_simulator.rsi_simulator import RSISimulator


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
        # Tests drive timer_callback() by hand; the node's own timer would
        # otherwise also fire during spin_once and add uncounted cycles.
        self.node.destroy_timer(self.node.timer)
        self.states = []
        self.node.create_subscription(
            String, f"{self.node.node_name_}/rsi/state", self._on_state, 10
        )

    def tearDown(self):
        self.node.destroy_node()
        self.fake_client.close()

    def _on_state(self, msg):
        root = ET.fromstring(msg.data)
        self.states.append(int(root.find("IPOC").text))

    def _run_cycles(self, count):
        """Drive the node's timer callback directly, bypassing the ROS timer.

        Each call either publishes synchronously or returns having skipped
        the cycle, so a bounded spin_once after it only ever flushes the one
        message that call may have produced; nothing here depends on gaps
        between invocations.
        """
        for _ in range(count):
            self.node.timer_callback()
            rclpy.spin_once(self.node, timeout_sec=0.5)

    def _set_fault(self, name, value):
        result = self.node.set_parameters([Parameter(name, value=value)])[0]
        self.assertTrue(result.successful, result.reason)

    def _assert_contiguous(self):
        self.assertEqual(
            self.states, list(range(self.states[0], self.states[0] + len(self.states)))
        )

    def test_default_behavior_is_unchanged(self):
        self._run_cycles(6)
        self.assertEqual(len(self.states), 6)
        self._assert_contiguous()

    def test_simulator_injects_late_packet(self):
        self._run_cycles(3)
        before = len(self.states)
        self._set_fault("fault_late_packet_cycles", 3)

        # The deferred cycles publish nothing: the count doesn't move.
        self._run_cycles(3)
        self.assertEqual(len(self.states), before)

        # The deferred send goes out on the next cycle, IPOCs stay contiguous.
        self._run_cycles(1)
        self.assertEqual(len(self.states), before + 1)
        self._assert_contiguous()

    def test_simulator_injects_dropped_packet(self):
        self._run_cycles(3)
        before = len(self.states)
        self._set_fault("fault_dropped_packet_count", 2)

        # The dropped cycles publish nothing: the count doesn't move.
        self._run_cycles(2)
        self.assertEqual(len(self.states), before)

        # Normal sending resumes on the next cycle, IPOCs stay contiguous.
        self._run_cycles(1)
        self.assertEqual(len(self.states), before + 1)
        self._assert_contiguous()

    def test_simulator_injects_repeated_ipoc(self):
        self._run_cycles(3)
        self._set_fault("fault_repeated_ipoc_count", 3)
        self._run_cycles(8)

        run_lengths = [len(list(group)) for _, group in itertools.groupby(self.states)]
        # The armed IPOC is sent once normally, then repeated 3 more times.
        self.assertTrue(4 in run_lengths, f"no run of 4 identical IPOCs, ipocs={self.states}")
        self.assertTrue(all(length == 1 for length in run_lengths if length != 4))


if __name__ == "__main__":
    unittest.main()
