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

"""
Loop timing when a hardware error moves the component to UNCONFIGURED, is_configured still true.

control_node's loop has no sleep on its is_configured branch, so this state busy-spins.
ros2_control_node, with hardware_synchronization.expect_blocking_read_write and a
minimum_cycle_time, sleeps every cycle regardless of hardware state.
"""

import os
import sys
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import psutil
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cpu_spin_support import (  # noqa: E402
    SPIN_THRESHOLD_PERCENT,
    measure_cpu_percent,
)
from mxa_status_fake_support import (  # noqa: E402
    COMPONENT,
    MXA_LOSS_LINE,
    LogCursor,
    MxaStatusFakeTestCase,
    robot_description,
)

# (hardware package, executable, extra controller_manager params). A bounded minimum cycle
# time only means something once a blocking/sleeping read-write is also declared.
NODE_VARIANTS = [
    ("kuka_drivers_core", "control_node", {}),
    (
        "controller_manager",
        "ros2_control_node",
        {
            "hardware_synchronization.expect_blocking_read_write": True,
            "hardware_synchronization.minimum_cycle_time": 0.001,
        },
    ),
]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
@launch_testing.parametrize("package, executable, extra_params", NODE_VARIANTS)
def generate_test_description(package, executable, extra_params):
    description = {"robot_description": robot_description()}

    control_node = launch_ros.actions.Node(
        package=package,
        executable=executable,
        output="screen",
        parameters=[
            description,
            {
                "update_rate": 250,
                "hardware_components_initial_state": {"inactive": [COMPONENT]},
            },
            extra_params,
        ],
    )

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[description],
    )

    # The mxA loss stops the simulator; respawn keeps a later test from inheriting it dead.
    simulator = launch_ros.actions.Node(
        package="kuka_rsi_simulator",
        executable="rsi_simulator",
        name="kuka_rsi_simulator",
        respawn=True,
        respawn_delay=0.2,
    )

    return launch.LaunchDescription(
        [
            control_node,
            robot_state_publisher,
            simulator,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class LoopInErroredUnconfiguredWithIsConfiguredStillTrue(MxaStatusFakeTestCase):
    def test_loop_in_errored_unconfigured_with_is_configured_still_true(
        self, proc_output, proc_info, package, executable, extra_params
    ):
        log = LogCursor(proc_output)
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        if not extra_params:
            # control_node gates read/update/write on this; ros2_control_node has no such gate.
            self.publish_is_configured()
        self.activate_and_stream(log)

        log = LogCursor(proc_output)
        self.inject("link_down")
        self.assertIsNotNone(
            log.wait_for_match(MXA_LOSS_LINE, timeout=5), "no mxA loss line in the driver log"
        )
        self.assertTrue(
            self._spin_until(
                lambda: self.component_state() == State.PRIMARY_STATE_UNCONFIGURED, timeout=10
            ),
            f"component did not error to UNCONFIGURED, it is {self.component_state()}",
        )

        pid = proc_info[executable].pid
        try:
            # Improve odds of a fair CPU share on a loaded runner, as in test_loop_timing.py's
            # negative control: a busy runner must not starve a real spin below the threshold.
            psutil.Process(pid).nice(-10)
        except psutil.AccessDenied:
            pass
        cpu = measure_cpu_percent(pid)
        print(f"{executable} CPU, errored UNCONFIGURED with is_configured still true: {cpu:.1f}%")

        if not extra_params:
            # control_node's is_configured branch has no sleep, so this state busy-spins; not
            # fixed on this branch.
            self.assertGreaterEqual(
                cpu,
                SPIN_THRESHOLD_PERCENT,
                f"{executable} did not spin at {cpu:.1f}% CPU "
                f"(threshold {SPIN_THRESHOLD_PERCENT}%)",
            )
            return

        self.assertLess(
            cpu,
            SPIN_THRESHOLD_PERCENT,
            f"{executable} busy-spins at {cpu:.1f}% CPU in errored UNCONFIGURED "
            f"(threshold {SPIN_THRESHOLD_PERCENT}%)",
        )


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
