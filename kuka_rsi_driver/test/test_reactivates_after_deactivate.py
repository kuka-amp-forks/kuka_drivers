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

"""After an SDK stop, the first command of the next activation is the measured position."""

import math
import os
import sys
import unittest
import xml.etree.ElementTree as ET

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from joint_move_test_case import MOVE_RAD  # noqa: E402
from rsi_fault_support import (  # noqa: E402
    SIMULATOR_COMMAND_LINE,
    SIMULATOR_STATE_TOPIC,
    STOP_REASON_CONTROL_STOPPED,
    STOP_REASON_NONE,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

AXES = [f"A{i}" for i in range(1, 7)]
# The simulator and the RSI corrections work in degrees, to six decimals or fewer.
TOLERANCE_DEG = 1e-3


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(respawn_simulator=True)


def _axes(xml_text, element):
    node = ET.fromstring(xml_text).find(element)
    return [float(node.attrib[axis]) for axis in AXES]


class TestReactivatesAfterDeactivate(RsiFaultTestCase):
    def setUp(self):
        super().setUp()
        self.measured = []
        self.node.create_subscription(
            String,
            SIMULATOR_STATE_TOPIC,
            lambda msg: self.measured.append(_axes(msg.data, "AIPos")),
            100,
        )

    def test_reactivates_after_deactivate(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)
        self.move_joint_and_wait()

        # The stop leaves JTC's last command, the moved position, in the command interface.
        log = LogCursor(proc_output)
        self.inject("stopped=RSI_program_stopped")
        self.assert_stopped(log, STOP_REASON_CONTROL_STOPPED)
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)
        commands = log.matches(SIMULATOR_COMMAND_LINE)
        self.assertTrue(commands, "no command before the stop")
        held = _axes(commands[-1].group(1), "AK")
        self.assertAlmostEqual(held[0], math.degrees(MOVE_RAD), delta=TOLERANCE_DEG)

        # The respawned simulator starts back at its home position, away from that command.
        self._spin_for(0.2)
        log = LogCursor(proc_output)
        first_state = len(self.measured)
        self.resume_and_activate(log)
        self.assertEqual(self.stop_reason(), STOP_REASON_NONE)
        commands = log.matches(SIMULATOR_COMMAND_LINE)
        self.assertTrue(commands, "no command after the activation")
        # AK is the correction from the position of the first packet, so 0 is the measured one.
        for axis, correction in zip(AXES, _axes(commands[0].group(1), "AK")):
            self.assertAlmostEqual(correction, 0.0, delta=TOLERANCE_DEG, msg=axis)
        states = self.measured[first_state:]
        self.assertGreater(len(states), 5, "too few simulator states after the activation")
        for state in states[1:]:
            for axis, start, now in zip(AXES, states[0], state):
                self.assertAlmostEqual(now, start, delta=TOLERANCE_DEG, msg=axis)

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
