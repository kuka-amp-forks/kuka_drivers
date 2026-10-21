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

"""The mxA component activates and follows a trajectory with no kroshu controller loaded."""

import os
import sys
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import SwitchController
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mxa_status_fake_support import (  # noqa: E402
    COMPONENT,
    OP_MODE_EXT,
    LogCursor,
    MxaStatusFakeTestCase,
    robot_description,
)


def _config(name):
    return os.path.join(get_package_share_directory("kuka_rsi_driver"), "config", name)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    description = {"robot_description": robot_description()}

    control_node = launch_ros.actions.Node(
        package="kuka_drivers_core",
        executable="control_node",
        output="screen",
        parameters=[
            description,
            _config("ros2_controller_config_rsi_only.yaml"),
            {"hardware_components_initial_state": {"inactive": [COMPONENT]}},
        ],
    )

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[description],
    )

    joint_state_broadcaster = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    joint_trajectory_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "controller_manager",
            "--param-file",
            _config("joint_trajectory_controller_config.yaml"),
            "--inactive",
        ],
    )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("kuka_rsi_simulator"),
                "/launch/",
                "kuka_rsi_simulator.launch.py",
            ]
        )
    )

    return launch.LaunchDescription(
        [
            control_node,
            robot_state_publisher,
            simulator,
            joint_state_broadcaster,
            joint_trajectory_controller,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestActivatesWithoutKroshuControllers(MxaStatusFakeTestCase):
    def _switch(self, activate=(), deactivate=()):
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        result = self._call(
            SwitchController, "/controller_manager/switch_controller", request, timeout=20
        )
        self.assertTrue(result.ok, f"switch_controller {activate} {deactivate} refused")

    def test_activates_without_kroshu_controllers(self, proc_output, proc_info):
        log = LogCursor(proc_output)
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)

        self._spin_for(3.0)
        names = set(self.node.get_node_names())
        self.assertTrue("controller_manager" in names, names)
        for kroshu in ("robot_manager", "control_mode_handler", "kss_message_handler"):
            self.assertFalse(kroshu in names, names)
        self.assertFalse("event_broadcaster" in names, names)

        self.publish_is_configured()
        self.inject(f"operation_mode={OP_MODE_EXT} drives_powered=1 motion_possible=1")

        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activate refused")
        self.assertTrue(log.wait_for("Starting joint position control at 4 ms", timeout=5))
        self.assertTrue(log.wait_for(f"Successful 'activate' of hardware '{COMPONENT}'", 30))

        self._switch(activate=["joint_trajectory_controller"])
        self.move_joint_and_wait()
        self._switch(deactivate=["joint_trajectory_controller"])
        self.deactivate_and_wait_for_simulator(proc_info)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
