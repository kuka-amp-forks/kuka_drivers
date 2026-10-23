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

"""Activation is refused at once on e-stop or outside EXT, and within its bound on dead drives."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mxa_status_fake_support import (  # noqa: E402
    OP_MODE_EXT,
    OP_MODE_T1,
    LogCursor,
    MxaStatusFakeTestCase,
    fake_launch_description,
    hardware_params,
)

DRIVES_ON_TIMEOUT_MS = 1500
# Covers the service round trip and the controller manager's transition thread on a slow
# runner; the lower bound is the driver's own wait, so the old 3 s constant would fail it.
MARGIN_S = 3.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fake_launch_description(
        extra_params=hardware_params(drives_on_timeout_ms=DRIVES_ON_TIMEOUT_MS), simulator=False
    )


class TestOnActivateRefusesWhenNotReady(MxaStatusFakeTestCase):
    def setUp(self):
        super().setUp()
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)

    def tearDown(self):
        self.inject("drives_stuck=0 emergency_stop=0")
        super().tearDown()

    def assert_refused_at_once(self, log, cause):
        ok, seconds = self.timed_set_component_state(State.PRIMARY_STATE_ACTIVE)
        self.assertFalse(ok, "activated")
        self.assertTrue(log.wait_for(cause, timeout=5), f"refusal does not name '{cause}'")
        self.assertLess(seconds, DRIVES_ON_TIMEOUT_MS / 1000, f"refusal took {seconds:.2f} s")
        self.assertFalse(log.wait_for("Turning on drives", timeout=0.1), "drives-on attempted")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)

    def test_estop_refused_at_once(self, proc_output):
        log = LogCursor(proc_output)
        self.inject(f"emergency_stop=1 operation_mode={OP_MODE_EXT} drives_powered=1")
        self.assert_refused_at_once(log, "Activation failed: Emergency stop is active")

    def test_non_ext_refused_at_once(self, proc_output):
        log = LogCursor(proc_output)
        self.inject(f"emergency_stop=0 operation_mode={OP_MODE_T1} drives_powered=1")
        self.assert_refused_at_once(log, "Activation failed: KRC not in EXT")

    def test_drives_not_powered_refused_within_bound(self, proc_output):
        log = LogCursor(proc_output)
        self.inject(
            f"emergency_stop=0 operation_mode={OP_MODE_EXT} drives_powered=0 drives_stuck=1"
        )
        ok, seconds = self.timed_set_component_state(State.PRIMARY_STATE_ACTIVE)
        self.assertFalse(ok, "activated with the drives off")
        bound_s = DRIVES_ON_TIMEOUT_MS / 1000
        self.assertGreaterEqual(seconds, bound_s, f"gave up on the drives after {seconds:.2f} s")
        self.assertLess(seconds, bound_s + MARGIN_S, f"drives-on wait took {seconds:.2f} s")
        self.assertTrue(
            log.wait_for(
                f"Activation failed: drives not powered within {DRIVES_ON_TIMEOUT_MS} ms",
                timeout=5,
            )
        )
        self.assertFalse(
            log.wait_for("Starting joint position control", timeout=0.1), "RSI start attempted"
        )
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
