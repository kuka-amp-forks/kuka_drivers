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

"""state/brake_test_due and state/brake_test_warning follow the mxA brake-test flags, live."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    JTC,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(respawn_simulator=True)


class TestExportsBrakeTestStatus(RsiFaultTestCase):
    def setUp(self):
        super().setUp()
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()

    def follows_flag(self, proc_output, token, name):
        """Check state/<name> follows the fake's <token> both ways, inactive and active."""
        self.assertEqual(self.state_value(name), 0.0)
        log = LogCursor(proc_output)
        self.inject(f"{token}=1")
        self.assertEqual(self.state_value(name), 1.0)
        self.inject(f"{token}=0")
        self.assertEqual(self.state_value(name), 0.0)

        # While active it is reported the same way and stops nothing.
        self.resume_and_activate(log)
        self.switch(activate=[JTC])
        log = LogCursor(proc_output)
        self.inject(f"{token}=1")
        self.assertEqual(self.state_value(name), 1.0)
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)
        self.assertEqual(self.controller_state(JTC), "active")
        self.assertFalse(log.wait_for("Stopping:", timeout=0.1), "a brake-test flag stopped it")
        self.inject(f"{token}=0")
        self.assertEqual(self.state_value(name), 0.0)

        self.switch(deactivate=[JTC])
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)

    def test_exports_brake_test_status(self, proc_output):
        self.follows_flag(proc_output, "brake_test_requested", "brake_test_due")

    def test_exports_brake_test_warning(self, proc_output):
        self.follows_flag(proc_output, "brake_test_warning", "brake_test_warning")


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
