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

"""The mxA and EKI hardware components export the six inhibitor slots and no stop_reason.

Run once per driver_version (see CMakeLists.txt); the component is only initialized, since
configuring it needs a live KRC. The slot semantics are covered on the mxA status fake by
test_inhibitor_slots.py.
"""

import unittest

import launch
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListHardwareInterfaces
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

COMPONENT = "kr6_r700_2"
INHIBITORS = ("estop", "guard", "mode", "drive_fault", "brake_test", "rsi")
SLOT_INTERFACES = ("active", "latched", "operator_action")


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("driver_version"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("kuka_rsi_driver"),
                        "/launch/",
                        "startup.launch.py",
                    ]
                ),
                launch_arguments={
                    "driver_version": LaunchConfiguration("driver_version"),
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestExportsInhibitorSlots(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_exports_inhibitor_slots(self, proc_output):
        proc_output.assertWaitFor(
            f"Successful initialization of hardware '{COMPONENT}'", timeout=30
        )

        node = rclpy.create_node("exports_inhibitor_slots_test")
        try:
            client = node.create_client(
                ListHardwareInterfaces, "/controller_manager/list_hardware_interfaces"
            )
            self.assertTrue(client.wait_for_service(timeout_sec=20), client.srv_name)
            future = client.call_async(ListHardwareInterfaces.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=20)
            self.assertIsNotNone(future.result(), f"{client.srv_name} did not answer")

            states = [i.name for i in future.result().state_interfaces]
            for name in INHIBITORS:
                for interface in SLOT_INTERFACES:
                    full = f"inhibitor/{name}/{interface}"
                    self.assertTrue(full in states, f"{full} not exported: {states}")
            self.assertFalse(
                [n for n in states if n.endswith("/stop_reason")],
                f"stop_reason still exported: {states}",
            )
            for status in ("state/drive_fault", "state/brake_test_due"):
                self.assertTrue(
                    [n for n in states if n.endswith(status)], f"{status} not exported: {states}"
                )
        finally:
            node.destroy_node()
