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

"""An e-stop pressed while the drives are being turned on fails activation at once."""

import os
import sys
import time
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from controller_manager_msgs.srv import SetHardwareComponentState
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    COMPONENT,
    OP_MODE_EXT,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

# The driver's default drives_on_timeout_ms.
DRIVES_ON_TIMEOUT_S = 5.0
TURNING_ON_LINE = "Turning on drives, waiting up to"
ESTOP_FAILURE_LINE = "Activation failed: Emergency stop is active after"
DRIVES_FAILED_LINE = "Activation failed: drives not powered within"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description()


class TestActivationFailsOnEstopDuringDrivesOn(RsiFaultTestCase):
    def test_activation_fails_on_estop_during_drives_on(self, proc_output):
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()
        self.inject(f"operation_mode={OP_MODE_EXT} motion_possible=1 drives_stuck=1")

        request = SetHardwareComponentState.Request()
        request.name = COMPONENT
        request.target_state.id = State.PRIMARY_STATE_ACTIVE
        client = self.node.create_client(
            SetHardwareComponentState, "/controller_manager/set_hardware_component_state"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=30), "no component state service")
        log = LogCursor(proc_output)
        future = client.call_async(request)
        self.assertTrue(log.wait_for(TURNING_ON_LINE, timeout=10), "drives-on never started")

        start = time.monotonic()
        self.inject("emergency_stop=1")
        result = self._wait_future(future, 2 * DRIVES_ON_TIMEOUT_S)
        elapsed = time.monotonic() - start
        self.assertFalse(result.ok, "activated under e-stop")
        self.assertLess(elapsed, DRIVES_ON_TIMEOUT_S / 2, "waited out the drives-on timeout")
        self.assertTrue(
            log.wait_for(ESTOP_FAILURE_LINE, timeout=1), "failure does not name e-stop"
        )
        self.assertFalse(log.wait_for(DRIVES_FAILED_LINE, timeout=0.1), "failed on the timeout")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)

        self.inject("emergency_stop=0 drives_stuck=0")


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
