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

"""Injected mxA status gates mxA component activation, and a frozen status stream withholds it."""

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


class TestMxaStatusFake(MxaStatusFakeTestCase):
    """Each test starts from a configured, inactive component in EXT with the drives off."""

    def setUp(self):
        super().setUp()
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()
        self.inject(
            "unfreeze_status emergency_stop=0 guard_stop=0 "
            f"operation_mode={OP_MODE_EXT} drives_powered=0 motion_possible=1"
        )

    def tearDown(self):
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        super().tearDown()

    def test_estop_refuses_activation(self, proc_output):
        log = LogCursor(proc_output)
        self.inject("emergency_stop=1")
        self.assertFalse(self.set_component_state(State.PRIMARY_STATE_ACTIVE))
        self.assertTrue(log.wait_for("Emergency stop is active", timeout=5))

    def test_non_ext_refuses_activation(self, proc_output):
        log = LogCursor(proc_output)
        self.inject(f"operation_mode={OP_MODE_T1}")
        self.assertFalse(self.set_component_state(State.PRIMARY_STATE_ACTIVE))
        self.assertTrue(log.wait_for("KRC not in EXT", timeout=5))

    def test_frozen_status_stream_withholds_status(self, proc_output):
        log = LogCursor(proc_output)
        self.inject(f"operation_mode={OP_MODE_T1}")
        self.inject("freeze_status")
        self.assertTrue(log.wait_for("mxA status fake: status frozen", timeout=5))
        self.inject(f"operation_mode={OP_MODE_EXT}")
        self.assertFalse(self.set_component_state(State.PRIMARY_STATE_ACTIVE))
        self.assertTrue(log.wait_for("KRC not in EXT", timeout=5))

    def test_unfreeze_status_delivers_status_and_activates(self, proc_output, proc_info):
        log = LogCursor(proc_output)
        self.inject(f"operation_mode={OP_MODE_T1}")
        self.inject("freeze_status")
        self.inject(f"operation_mode={OP_MODE_EXT}")
        self.inject("unfreeze_status")
        self.assertTrue(log.wait_for("mxA status fake: status unfrozen", timeout=5))
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_ACTIVE))
        self.assertTrue(log.wait_for("Drives successfully powered on", timeout=5))
        self.assertTrue(log.wait_for(f"Successful 'activate' of hardware '{COMPONENT}'", 30))
        self.deactivate_and_wait_for_simulator(proc_info)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
