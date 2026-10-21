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

"""The first activation after configure receives its first packet and leaves RSI running."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mxa_status_fake_support import (  # noqa: E402
    COMPONENT,
    FAKE_CANCEL_REQUESTED,
    FIRST_PACKET_FAILURE,
    OP_MODE_EXT,
    RECEIVED_FIRST_PACKET,
    LogCursor,
    MxaStatusFakeTestCase,
    fake_launch_description,
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fake_launch_description()


class TestFirstActivationReceivesFirstPacket(MxaStatusFakeTestCase):
    def test_first_activation_receives_first_packet(self, proc_output, proc_info):
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        log = LogCursor(proc_output)
        self.inject(f"operation_mode={OP_MODE_EXT} drives_powered=1 motion_possible=1")

        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activate refused")
        self.assertTrue(
            log.wait_for(
                "Starting joint position control at 4 ms, waiting up to 5000 ms", timeout=5
            )
        )
        self.assertTrue(log.wait_for(RECEIVED_FIRST_PACKET, timeout=5), "no first-packet line")
        self.assertTrue(log.wait_for(f"Successful 'activate' of hardware '{COMPONENT}'", 30))
        self.assertFalse(log.wait_for(FIRST_PACKET_FAILURE, timeout=0.1), "activation failed")
        self.assertFalse(log.wait_for(FAKE_CANCEL_REQUESTED, timeout=0.1), "RSI cancelled")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)

        report = self.fake_report(proc_output)
        self.assertEqual(report.group(1), "1", "RSI context not running")
        self.assertEqual(report.group(2), "0", "cancel pending")

        self.publish_is_configured()
        self.deactivate_and_wait_for_simulator(proc_info)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
