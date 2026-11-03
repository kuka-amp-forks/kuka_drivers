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

"""A switch to joint velocity is refused in prepare_command_mode_switch; a GPIO command is not."""

import os
import re
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import SwitchController
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    JTC,
    STOP_REASON_NONE,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

VELOCITY_CONTROLLER = "velocity_command_controller"
GPIO_CONTROLLER = "gpio_command_controller"
GPIO_CONFIG = os.path.join(
    get_package_share_directory("kuka_rsi_driver"), "test", "config", "gpio_command_only.xacro"
)
REFUSED_LINE = re.compile(
    r"Refusing the command mode switch: '([^']+)' requested, only joint position is supported"
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(
        respawn_simulator=True,
        inactive_controllers=[
            (VELOCITY_CONTROLLER, "velocity_command_controller.yaml"),
            (GPIO_CONTROLLER, "gpio_command_controller.yaml"),
        ],
        gpio_config=GPIO_CONFIG,
    )


class TestRefusesUnsupportedModeSwitch(RsiFaultTestCase):
    def finish(self, log, *controllers):
        self.switch(deactivate=list(controllers))
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)

    def test_refuses_unsupported_mode_switch(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)
        self.assertEqual(self.controller_state(VELOCITY_CONTROLLER), "inactive")

        log = LogCursor(proc_output)
        request = SwitchController.Request()
        request.activate_controllers = [VELOCITY_CONTROLLER]
        request.deactivate_controllers = [JTC]
        request.strictness = SwitchController.Request.STRICT
        result = self._call(
            SwitchController, "/controller_manager/switch_controller", request, timeout=20
        )
        self.assertFalse(result.ok, "switch to joint velocity accepted")
        match = log.wait_for_match(REFUSED_LINE, timeout=5)
        self.assertIsNotNone(match, "not refused in prepare_command_mode_switch")
        self.assertRegex(match.group(1), r"^joint_[1-6]/velocity$")

        self.assertEqual(self.controller_state(JTC), "active")
        self.assertEqual(self.controller_state(VELOCITY_CONTROLLER), "inactive")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)
        self.assertEqual(self.stop_reason(), STOP_REASON_NONE)
        self.move_joint_and_wait()
        self.finish(log, JTC)

    def test_accepts_gpio_command_switch(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)
        self.assertEqual(self.controller_state(GPIO_CONTROLLER), "inactive")

        log = LogCursor(proc_output)
        self.switch(activate=[GPIO_CONTROLLER])
        self.assertEqual(log.matches(REFUSED_LINE), [], "GPIO command switch refused")
        self.assertEqual(self.controller_state(GPIO_CONTROLLER), "active")
        self.assertEqual(self.controller_state(JTC), "active")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)
        self.assertEqual(self.stop_reason(), STOP_REASON_NONE)
        self.move_joint_and_wait()
        self.finish(log, GPIO_CONTROLLER, JTC)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
