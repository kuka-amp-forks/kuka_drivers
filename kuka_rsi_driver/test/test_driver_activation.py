# Copyright 2024 KUKA Hungaria Kft.
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

"""robot_manager brings the rsi_only component up on the simulator and a trajectory runs."""

import os
import sys

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
from ament_index_python.packages import get_package_share_directory
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from joint_move_test_case import JointMoveTestCase  # noqa: E402
from rsi_only_support import COMPONENT, rsi_only_description  # noqa: E402


def _config(name):
    return os.path.join(get_package_share_directory("kuka_rsi_driver"), "config", name)


def _spawner(name, param_file=None):
    arguments = [name, "-c", "controller_manager"]
    if param_file:
        arguments += ["--param-file", param_file]
    return launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=arguments + ["--inactive"],
    )


# The nodes of startup.launch.py for rsi_only, with the simulator's read timeout in the
# description, which the stock xacro cannot carry.
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    description = {"robot_description": rsi_only_description()}

    control_node = launch_ros.actions.Node(
        package="kuka_drivers_core",
        executable="control_node",
        parameters=[
            description,
            _config("ros2_controller_config_rsi_only.yaml"),
            {"hardware_components_initial_state": {"unconfigured": [COMPONENT]}},
        ],
    )
    robot_manager = launch_ros.actions.LifecycleNode(
        name="robot_manager",
        namespace="",
        package="kuka_rsi_driver",
        executable="robot_manager_node_rsi_only",
        parameters=[_config("driver_config.yaml"), {"robot_models": [COMPONENT]}],
    )
    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[description],
    )

    return launch.LaunchDescription(
        [
            control_node,
            robot_manager,
            robot_state_publisher,
            _spawner("joint_state_broadcaster"),
            _spawner(
                "joint_trajectory_controller", _config("joint_trajectory_controller_config.yaml")
            ),
            _spawner("event_broadcaster", _config("kuka_event_broadcaster_config.yaml")),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("kuka_rsi_simulator"),
                        "/launch/",
                        "kuka_rsi_simulator.launch.py",
                    ]
                )
            ),
            launch_ros.actions.Node(
                package="kuka_drivers_core",
                executable="lifecycle_manager",
                parameters=[
                    {
                        "managed_node": "robot_manager",
                        "configure_delay": 20.0,
                        "activate_delay": 25.0,
                    }
                ],
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestDriverActivation(JointMoveTestCase):
    def test_read_stdout(self, proc_output):
        # Check for successful initialization
        proc_output.assertWaitFor("Robot initialized", timeout=5)
        proc_output.assertWaitFor(
            f"Successful initialization of hardware '{COMPONENT}'", timeout=5
        )
        # Check whether disabling automatic activation was successful
        proc_output.assertWaitFor(
            f"Setting component '{COMPONENT}' to 'unconfigured' state.", timeout=5
        )
        # Check for successful configuration and activation
        proc_output.assertWaitFor(f"Successful 'configure' of hardware '{COMPONENT}'", timeout=25)
        proc_output.assertWaitFor(f"Successful 'activate' of hardware '{COMPONENT}'", timeout=30)

        self._wait_for_controller_active("joint_trajectory_controller", timeout=20)
        self.move_joint_and_wait()
