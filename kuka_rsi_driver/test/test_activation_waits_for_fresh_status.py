# Copyright 2026 KUKA Hungaria Kft.
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

"""After an mxA loss and a reconfigure, activation decides on the new link's status only."""

import os
import re
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from controller_manager_msgs.srv import SetHardwareComponentState
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    COMPONENT,
    JSB,
    MXA_RELEASED_LINE,
    INHIBITOR_BROADCASTER,
    OP_MODE_EXT,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
    hardware_params,
)

# Well past the 0.3 s the test waits before the new link's first status.
FIRST_STATUS_TIMEOUT_MS = 3000
NO_STATUS_LINE = (
    f"Activation failed: no controller status since configure within {FIRST_STATUS_TIMEOUT_MS} ms"
)
FRESH_STATUS_LINE = re.compile(r"mxA status fake: operation_mode=4 motion_possible=1")
DRIVES_ON_LINE = re.compile(r"Turning on drives, waiting up to \d+ ms")
STARTING_LINE = "Starting joint position control at 4 ms"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(
        respawn_simulator=True,
        extra_params=hardware_params(first_status_timeout_ms=FIRST_STATUS_TIMEOUT_MS),
    )


class TestActivationWaitsForFreshStatus(RsiFaultTestCase):
    def test_activation_waits_for_fresh_status(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)

        # The old link last reported EXT with the drives on.
        log = LogCursor(proc_output)
        self.inject("link_down")
        self.assertTrue(log.wait_for(MXA_RELEASED_LINE, timeout=5), "client not released")
        self.wait_for_component_state(State.PRIMARY_STATE_UNCONFIGURED)
        self.wait_for_simulator_respawn(log)

        # The new fake reports nothing until its status changes, so the first status is delayed.
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "reconfigure")
        self.switch(activate=[JSB, INHIBITOR_BROADCASTER])
        # The old link's EXT is gone with it: the mode is unknown until the new link reports.
        self.assert_active({"mode"})

        log = LogCursor(proc_output)
        self.assertFalse(self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activated")
        self.assertTrue(log.wait_for(NO_STATUS_LINE, timeout=1), "not refused for no status")
        self.assertIsNone(log.wait_for_match(DRIVES_ON_LINE, timeout=0.1))
        self.assertFalse(log.wait_for(STARTING_LINE, timeout=0.1), "RSI started on old status")

        log = LogCursor(proc_output)
        request = SetHardwareComponentState.Request()
        request.name = COMPONENT
        request.target_state.id = State.PRIMARY_STATE_ACTIVE
        client = self.node.create_client(
            SetHardwareComponentState, "/controller_manager/set_hardware_component_state"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=30), "no component state service")
        future = client.call_async(request)
        self._spin_for(0.3)
        self.inject(f"operation_mode={OP_MODE_EXT} motion_possible=1")
        self.assertTrue(log.wait_for(STARTING_LINE, timeout=10), "no start")
        self.set_simulator_fault("fault_dropped_packet_count", 0)
        self.assertTrue(self._wait_future(future, 60).ok, "activate refused")

        fresh = log.wait_for_match(FRESH_STATUS_LINE, timeout=1)
        drives_on = log.wait_for_match(DRIVES_ON_LINE, timeout=1)
        self.assertIsNotNone(fresh, "no fresh status injected")
        self.assertIsNotNone(drives_on, "drives not turned on: activated on the old status")
        self.assertLess(fresh.start(), drives_on.start(), "drives decided before the new status")

        self._spin_for(1.0)
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)
        self.assert_active(set())
        self.assert_latched(set())

        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
