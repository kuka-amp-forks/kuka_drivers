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

"""A failed RSI send deactivates the component instead of throwing; only a stop packet follows."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mxa_status_fake_support import DEACTIVATION_LINE  # noqa: E402
from rsi_fault_support import (  # noqa: E402
    HARDWARE_EVENT_ERROR,
    SEND_FAILURE_LINE,
    STOP_REASON_RSI_SEND_FAILED,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description()


class TestDeactivatesOnSendFailure(RsiFaultTestCase):
    def test_deactivates_on_send_failure(self, proc_output, proc_info):
        log = LogCursor(proc_output)
        self.start_streaming(log)

        log = LogCursor(proc_output)
        self.hardware_events.clear()
        self.inject("send_fail=1")
        match = log.wait_for_match(SEND_FAILURE_LINE, timeout=5)
        self.assertIsNotNone(match, "no stop line in the driver log")
        failed_ipoc = int(match.group(1))
        # A send failure reaches the event broadcaster the same as a missed packet, so
        # robot_manager hears about it even though write() never throws.
        self.assertTrue(
            self._spin_until(lambda: HARDWARE_EVENT_ERROR in self.hardware_events, timeout=5),
            f"no error event on the event broadcaster, saw {self.hardware_events}",
        )
        self.assert_stopped(log, STOP_REASON_RSI_SEND_FAILED)
        self.assertFalse(log.wait_for("Quitting control loop", timeout=0.1), "control loop died")

        # The pending request is answered with the stop packet, nothing else.
        self.assertTrue(log.wait_for_match(DEACTIVATION_LINE, timeout=5), "no stop packet line")
        proc_info.assertWaitForShutdown(process="rsi_simulator", timeout=30)
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        commands = self.logged_commands(log)
        self.assertEqual(commands[-1], (failed_ipoc, True), commands)
        self.assertTrue(all(i < failed_ipoc and not stop for i, stop in commands[:-1]), commands)
        topic_commands = self.commands_after(failed_ipoc - 1)
        self.assertTrue(topic_commands in ([], [(failed_ipoc, True)]), topic_commands)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
