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

"""Activation reads the live mxA status, not the copy the last read() made."""

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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mxa_status_fake_support import (  # noqa: E402
    COMPONENT,
    OP_MODE_EXT,
    OP_MODE_T1,
    LogCursor,
    MxaStatusFakeTestCase,
    robot_description,
)


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
            {
                "update_rate": 250,
                "hardware_components_initial_state": {"unconfigured": [COMPONENT]},
            },
        ],
    )

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[description],
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
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestOnActivateSeesFreshStatus(MxaStatusFakeTestCase):
    """control_node runs read() only after is_configured, which is published last here."""

    def test_on_activate_sees_fresh_status(self, proc_output, proc_info):
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)

        log = LogCursor(proc_output)
        self.inject(f"operation_mode={OP_MODE_T1} drives_powered=1 motion_possible=1")
        self.assertFalse(self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activated in T1")
        self.assertTrue(log.wait_for("KRC not in EXT", timeout=5))
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)

        log = LogCursor(proc_output)
        self.inject(f"operation_mode={OP_MODE_EXT}")
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activate refused")
        self.assertTrue(log.wait_for(f"Successful 'activate' of hardware '{COMPONENT}'", 30))
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)

        self.publish_is_configured()
        self.deactivate_and_wait_for_simulator(proc_info)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
