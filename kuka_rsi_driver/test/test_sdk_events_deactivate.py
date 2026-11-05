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

"""The SDK's stop and error events stop an active component; its mxA timeout error does not."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    CONTROL_EVENT_STOP_LINE,
    DEACTIVATION_REQUESTS_LINE,
    EVENT_BROADCASTER,
    HARDWARE_EVENT_ERROR,
    JSB,
    JTC_DEACTIVATED_LINE,
    RECEIVE_FAILURE_LINE,
    INHIBITOR_BROADCASTER,
    SDK_ERROR_LINE,
    SDK_RECEIVE_TIMEOUT_TEXT,
    UNATTRIBUTED_LINE,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(respawn_simulator=True)


class TestSdkEvents(RsiFaultTestCase):
    def setUp(self):
        super().setUp()
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()

    def stop_on_event(self, proc_output, datagram, name, text, latched, unattributed):
        """Stream, raise an SDK event, check it stops the component with its slot, reactivate."""
        log = LogCursor(proc_output)
        self.start_streaming(log)

        log = LogCursor(proc_output)
        self.hardware_events.clear()
        self.inject(datagram)
        match = log.wait_for_match(CONTROL_EVENT_STOP_LINE, timeout=5)
        self.assertIsNotNone(match, "no stop line in the driver log")
        self.assertEqual(match.group(1), name)
        self.assertEqual(match.group(2), text)
        self.assert_stopped(log, latched)
        self.assertTrue(
            self._spin_until(lambda: HARDWARE_EVENT_ERROR in self.hardware_events, timeout=5),
            f"no error event on the event broadcaster, saw {self.hardware_events}",
        )
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.assertEqual(log.matches(RECEIVE_FAILURE_LINE), [], "RSI blamed for an SDK event")
        self.assertEqual(len(log.matches(UNATTRIBUTED_LINE)), 1 if unattributed else 0)
        self.wait_for_simulator_respawn(log)

        log = LogCursor(proc_output)
        self.resume_and_activate(log)
        self.assert_latched(set())
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for("Stop complete", timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)

    def test_sdk_stop_event_deactivates(self, proc_output):
        # No flag explains a program stop, so it is the unattributed stop: rsi.
        self.stop_on_event(
            proc_output,
            "stopped=RSI_program_stopped",
            "external control stopped",
            "RSI_program_stopped",
            latched={"rsi"},
            unattributed=True,
        )

    def test_sdk_error_event_deactivates(self, proc_output):
        # An SDK error other than the stop message (mxA error 801, covered by
        # test_exports_status_state_interfaces) explains no slot, so it is unattributed too.
        self.stop_on_event(
            proc_output,
            "error=Keep-alive_thread_error_529",
            "external control error",
            "Keep-alive_thread_error_529",
            latched={"rsi"},
            unattributed=True,
        )

    def test_sdk_events_do_not_latch_while_inactive(self, proc_output):
        held = self.latched()
        log = LogCursor(proc_output)
        self.inject("stopped=RSI_program_cancelled error=Keep-alive_thread_error_529")
        self.assertIsNotNone(log.wait_for_match(SDK_ERROR_LINE, timeout=5), "no error event")
        self._spin_for(0.5)
        self.assertEqual(
            log.matches(CONTROL_EVENT_STOP_LINE), [], "SDK event latched while inactive"
        )
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_INACTIVE)
        self.assertFalse("rsi" in self.active(), "rsi raised while inactive")
        self.assertEqual(self.latched(), held)

    def test_mxa_timeout_error_does_not_deactivate(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)

        log = LogCursor(proc_output)
        self.inject("link_down")
        self.wait_for_component_state(State.PRIMARY_STATE_UNCONFIGURED)
        match = log.wait_for_match(SDK_ERROR_LINE, timeout=5)
        self.assertIsNotNone(match, "the SDK's receive-timeout error was not raised")
        self.assertEqual(match.group(1), SDK_RECEIVE_TIMEOUT_TEXT)
        self.assertEqual(log.matches(CONTROL_EVENT_STOP_LINE), [], "timeout error latched a stop")
        self.assertFalse(log.wait_for(DEACTIVATION_REQUESTS_LINE, timeout=0.1), "deactivated")
        self.assertFalse(log.wait_for(JTC_DEACTIVATED_LINE, timeout=0.1), "deactivated")
        # on_error answered the last packet with the stop flag.
        self.wait_for_simulator_respawn(log)

        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "reconfigure")
        self.switch(activate=[JSB, INHIBITOR_BROADCASTER, EVENT_BROADCASTER])
        # The loss has no slot.
        self.assert_latched(set())


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
