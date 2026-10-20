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

"""The hardware component follows a trajectory with no kroshu controller loaded."""

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
from controller_manager_msgs.srv import (
    ListHardwareInterfaces,
    SetHardwareComponentState,
    SwitchController,
)
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from lifecycle_msgs.msg import State
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from joint_move_test_case import JointMoveTestCase  # noqa: E402

COMPONENT = "kr6_r700_2"


def _config(name):
    return os.path.join(get_package_share_directory("kuka_rsi_driver"), "config", name)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    get_package_share_directory("kuka_agilus_support"),
                    "urdf",
                    f"{COMPONENT}.urdf.xacro",
                ]
            ),
            " driver_version:=rsi_only client_ip:=0.0.0.0 client_port:=59152 roundtrip_time:=4000",
        ],
        on_stderr="capture",
    )

    control_node = launch_ros.actions.Node(
        package="kuka_drivers_core",
        executable="control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            _config("ros2_controller_config_rsi_only.yaml"),
            {"hardware_components_initial_state": {"inactive": [COMPONENT]}},
        ],
    )

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
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


class TestStreamsWithoutEventBroadcaster(JointMoveTestCase):
    def test_follows_trajectory_without_event_broadcaster(self, proc_output):
        proc_output.assertWaitFor(f"Successful 'configure' of hardware '{COMPONENT}'", timeout=30)

        self._spin_for(3.0)
        names = set(self.node.get_node_names())
        self.assertTrue("controller_manager" in names, names)
        self.assertFalse("robot_manager" in names, names)
        self.assertFalse("event_broadcaster" in names, names)

        interfaces = self._call(
            ListHardwareInterfaces,
            "/controller_manager/list_hardware_interfaces",
            ListHardwareInterfaces.Request(),
            timeout=20,
        )
        exported = [i.name for i in interfaces.command_interfaces + interfaces.state_interfaces]
        self.assertFalse(
            [n for n in exported if "interpolation_count" in n],
            f"interpolation_count still exported: {exported}",
        )

        # control_node runs read() and write() only after robot_manager/is_configured is true.
        is_configured = self.node.create_publisher(
            Bool,
            "/robot_manager/is_configured",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.assertTrue(
            self._spin_until(lambda: is_configured.get_subscription_count() > 0, timeout=10),
            "control_node is not subscribed to /robot_manager/is_configured",
        )
        for _ in range(5):
            is_configured.publish(Bool(data=True))
            self._spin_for(0.1)

        request = SetHardwareComponentState.Request()
        request.name = COMPONENT
        request.target_state.id = State.PRIMARY_STATE_ACTIVE
        request.target_state.label = "active"
        result = self._call(
            SetHardwareComponentState,
            "/controller_manager/set_hardware_component_state",
            request,
            timeout=60,
        )
        self.assertTrue(result.ok, "set_hardware_component_state(active) refused")
        proc_output.assertWaitFor(f"Successful 'activate' of hardware '{COMPONENT}'", timeout=30)

        switch = SwitchController.Request()
        switch.activate_controllers = ["joint_trajectory_controller"]
        switch.strictness = SwitchController.Request.STRICT
        result = self._call(
            SwitchController, "/controller_manager/switch_controller", switch, timeout=20
        )
        self.assertTrue(result.ok, "joint_trajectory_controller did not activate")

        self.move_joint_and_wait()

        switch = SwitchController.Request()
        switch.deactivate_controllers = ["joint_trajectory_controller"]
        switch.strictness = SwitchController.Request.STRICT
        result = self._call(
            SwitchController, "/controller_manager/switch_controller", switch, timeout=20
        )
        self.assertTrue(result.ok, "joint_trajectory_controller did not deactivate")

        request.target_state.id = State.PRIMARY_STATE_INACTIVE
        request.target_state.label = "inactive"
        result = self._call(
            SetHardwareComponentState,
            "/controller_manager/set_hardware_component_state",
            request,
            timeout=30,
        )
        self.assertTrue(result.ok, "set_hardware_component_state(inactive) refused")


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
