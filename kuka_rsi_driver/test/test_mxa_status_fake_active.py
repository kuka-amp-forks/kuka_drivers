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

"""While active, read() reports an injected mxA condition. One launch per condition."""

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
    LogCursor,
    MxaStatusFakeTestCase,
    robot_description,
)

# The RSI simulator exits on the stop flag, so each activation needs its own launch.
CONDITIONS = [
    ("motion_possible=0", "Motion is not possible"),
    ("emergency_stop=1", "Emergency stop detected!"),
]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
@launch_testing.parametrize("condition, message", CONDITIONS)
def generate_test_description(condition, message):
    description = {"robot_description": robot_description()}

    control_node = launch_ros.actions.Node(
        package="kuka_drivers_core",
        executable="control_node",
        output="screen",
        parameters=[
            description,
            {
                "update_rate": 250,
                "hardware_components_initial_state": {"inactive": [COMPONENT]},
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


class TestMxaStatusFakeActive(MxaStatusFakeTestCase):
    def setUp(self):
        super().setUp()
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()
        self.inject(f"operation_mode={OP_MODE_EXT} drives_powered=1 motion_possible=1")

    def tearDown(self):
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        super().tearDown()

    def test_read_reports_condition(self, proc_output, condition, message):
        log = LogCursor(proc_output)
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activate refused")
        self.assertTrue(log.wait_for(f"Successful 'activate' of hardware '{COMPONENT}'", 30))
        self.inject(condition)
        self.assertTrue(log.wait_for(message, timeout=5), f"{condition} did not log {message!r}")


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
