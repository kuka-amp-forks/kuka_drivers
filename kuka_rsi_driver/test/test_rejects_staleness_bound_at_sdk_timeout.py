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

"""An mxA staleness bound at or above the SDK's 100 ms receive timeout is refused at configure."""

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
    LogCursor,
    MxaStatusFakeTestCase,
    fake_launch_description,
    hardware_params,
)

SDK_TIMEOUT_MS = 100


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fake_launch_description(
        extra_params=hardware_params(mxa_status_timeout_ms=SDK_TIMEOUT_MS),
        simulator=False,
        initial_state="unconfigured",
    )


class TestRejectsStalenessBoundAtSdkTimeout(MxaStatusFakeTestCase):
    def test_rejects_staleness_bound_at_sdk_timeout(self, proc_output):
        proc_output.assertWaitFor(
            f"Successful initialization of hardware '{COMPONENT}'", timeout=30
        )
        proc_output.assertWaitFor(f"mxA status timeout: {SDK_TIMEOUT_MS} ms", timeout=5)

        log = LogCursor(proc_output)
        self.assertFalse(
            self.set_component_state(State.PRIMARY_STATE_INACTIVE),
            "configured with the bound at the SDK timeout",
        )
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_UNCONFIGURED)
        self.assertTrue(
            log.wait_for(
                f"mxa_status_timeout_ms is {SDK_TIMEOUT_MS}, it must stay below the SDK's "
                f"{SDK_TIMEOUT_MS} ms mxA receive timeout",
                timeout=5,
            )
        )


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
