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

"""A stop pending on a frozen status stream waits; losing the link while INACTIVE errors instead."""

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
    MXA_LOSS_LINE,
    STOP_COMPLETE_LINE,
    LogCursor,
    MxaStatusFakeTestCase,
    fake_launch_description,
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fake_launch_description(respawn_simulator=True)


class TestDeactivateReturnsWithMxaLinkDown(MxaStatusFakeTestCase):
    def test_deactivate_returns_with_mxa_link_down(self, proc_output):
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()
        log = LogCursor(proc_output)
        self.activate_and_stream(log)

        log = LogCursor(proc_output)
        self.inject("freeze_status")
        self.assert_deactivation_within_one_cycle(log)

        # With the status withheld neither the cancel nor the drives-off can be confirmed.
        self.assertFalse(
            log.wait_for(STOP_COMPLETE_LINE, timeout=1.0),
            "stop completed with the status withheld",
        )

        # Losing the link while the stop is pending is an error, not a longer wait.
        log = LogCursor(proc_output)
        self.inject("link_down")
        self.assertIsNotNone(log.wait_for_match(MXA_LOSS_LINE, timeout=5), "no mxA loss line")
        self.assertTrue(
            self._spin_until(
                lambda: self.component_state() == State.PRIMARY_STATE_UNCONFIGURED, timeout=10
            ),
            "component did not reach UNCONFIGURED",
        )

        # A reconfigure reconnects and the next activation is not refused by the old stop.
        log = LogCursor(proc_output)
        self.inject("link_up resume")
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "reconfigure")
        self.activate_and_stream(log)
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for(STOP_COMPLETE_LINE, timeout=5), "stop never completed")
        self.assertTrue(log.wait_for(f"Successful 'deactivate' of hardware '{COMPONENT}'", 5))
        self.wait_for_simulator_respawn(log)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
