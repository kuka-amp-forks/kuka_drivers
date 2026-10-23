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

"""Configuring the mxA component with no mxA server fails within configure_timeout_ms."""

import os
import sys
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    SetHardwareComponentState,
)
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from joint_move_test_case import JointMoveTestCase  # noqa: E402

COMPONENT = "kr6_r700_2"
CONTROLLER_IP = "127.0.0.1"
CONFIGURE_TIMEOUT_MS = 1000
MARGIN_S = 10.0


def _robot_description():
    urdf = os.path.join(
        get_package_share_directory("kuka_agilus_support"), "urdf", f"{COMPONENT}.urdf.xacro"
    )
    description = xacro.process_file(
        urdf,
        mappings={
            "driver_version": "mxa_rsi",
            "client_ip": CONTROLLER_IP,
            "controller_ip": CONTROLLER_IP,
        },
    ).toxml()
    anchor = f'<param name="controller_ip">{CONTROLLER_IP}</param>'
    assert anchor in description, description
    return description.replace(
        anchor, anchor + f'<param name="configure_timeout_ms">{CONFIGURE_TIMEOUT_MS}</param>', 1
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    robot_description = _robot_description()

    control_node = launch_ros.actions.Node(
        package="kuka_drivers_core",
        executable="control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(
                get_package_share_directory("kuka_rsi_driver"),
                "config",
                "ros2_controller_config_extended.yaml",
            ),
            {"hardware_components_initial_state": {"unconfigured": [COMPONENT]}},
        ],
    )

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    return launch.LaunchDescription(
        [control_node, robot_state_publisher, launch_testing.actions.ReadyToTest()]
    )


class TestConfigureTimesOutWithoutMxa(JointMoveTestCase):
    def test_configure_times_out_without_mxa(self, proc_output):
        proc_output.assertWaitFor(
            f"Successful initialization of hardware '{COMPONENT}'", timeout=30
        )

        request = SetHardwareComponentState.Request()
        request.name = COMPONENT
        request.target_state.id = State.PRIMARY_STATE_INACTIVE
        request.target_state.label = "inactive"
        bound_s = CONFIGURE_TIMEOUT_MS / 1000
        start = time.monotonic()
        result = self._call(
            SetHardwareComponentState,
            "/controller_manager/set_hardware_component_state",
            request,
            timeout=bound_s + MARGIN_S,
        )
        elapsed = time.monotonic() - start
        self.assertFalse(result.ok, "configure succeeded with no mxA server")
        self.assertLess(elapsed, bound_s + MARGIN_S, f"configure returned after {elapsed:.1f} s")
        self.assertEqual(result.state.id, State.PRIMARY_STATE_UNCONFIGURED, result.state.label)
        proc_output.assertWaitFor(
            f"No initialization report from the mxA server at {CONTROLLER_IP} "
            f"within {CONFIGURE_TIMEOUT_MS} ms",
            timeout=5,
        )

        components = self._call(
            ListHardwareComponents,
            "/controller_manager/list_hardware_components",
            ListHardwareComponents.Request(),
            timeout=10,
        )
        states = {c.name: c.state.id for c in components.component}
        self.assertEqual(states.get(COMPONENT), State.PRIMARY_STATE_UNCONFIGURED, states)

        controllers = self._call(
            ListControllers,
            "/controller_manager/list_controllers",
            ListControllers.Request(),
            timeout=10,
        )
        self.assertIsNotNone(controllers)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
