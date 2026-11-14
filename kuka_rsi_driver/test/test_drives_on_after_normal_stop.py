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

"""Drives come on through the KRC's Normal Stop after a drives-off, also after a failed attempt."""

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
    OP_MODE_EXT,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

# The driver's default drives_on_timeout_ms, which must cover the Normal Stop's release.
DRIVES_ON_TIMEOUT_S = 5.0
# How long the fake's Normal Stop takes to release after a drives-on request, as on the r930.
RELEASE_MS = 1900
# Longer than a whole drives-on attempt.
SLOW_RELEASE_MS = 7000

DRIVES_POWERED_LINE = re.compile(r"Drives successfully powered on after (\d+) ms")
DRIVES_FAILED_LINE = re.compile(r"Activation failed: drives not powered within \d+ ms of (\d+)")
PERI_FLASH_LINE = "mxA status fake: PERI_RDY flash with guard_stop=1"
STOP_COMPLETE_LINE = "Stop complete"
STARTING_LINE = "Starting joint position control at 4 ms"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(respawn_simulator=True)


class TestDrivesOnAfterNormalStop(RsiFaultTestCase):
    def request_active(self):
        request = SetHardwareComponentState.Request()
        request.name = COMPONENT
        request.target_state.id = State.PRIMARY_STATE_ACTIVE
        client = self.node.create_client(
            SetHardwareComponentState, "/controller_manager/set_hardware_component_state"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=30), "no component state service")
        return client.call_async(request)

    def deactivate_into_normal_stop(self, proc_output, release_ms):
        """Deactivate with the drives on, so the fake holds a Normal Stop after the drives-off."""
        self.inject(f"normal_stop=1 normal_stop_release_ms={release_ms}")
        log = LogCursor(proc_output)
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for(STOP_COMPLETE_LINE, timeout=5), "stop never completed")
        self.assertEqual(self.state_value("guard_stop"), 1.0)
        return log

    def activate_with_simulator(self, log, future, respawned=False):
        if not respawned:
            self.wait_for_simulator_respawn(log)
        self.assertTrue(log.wait_for(STARTING_LINE, timeout=10), "no RSI start")
        self.set_simulator_fault("fault_dropped_packet_count", 0)
        self.assertTrue(self._wait_future(future, 60).ok, "activate refused")

    def test_drives_on_after_normal_stop(self, proc_output):
        # Drives are on through the fake's own edge, not an injected flag.
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()
        self.inject(f"operation_mode={OP_MODE_EXT} motion_possible=1")
        log = LogCursor(proc_output)
        future = self.request_active()
        self.assertTrue(log.wait_for(STARTING_LINE, timeout=10), "no RSI start")
        self.set_simulator_fault("fault_dropped_packet_count", 0)
        self.assertTrue(self._wait_future(future, 60).ok, "first activate refused")
        self.assertIsNotNone(log.wait_for_match(DRIVES_POWERED_LINE, timeout=1), "not powered")

        # Activate right after the stop: the first request releases the Normal Stop, and only a
        # later edge, once operator safety is back, switches the drives on.
        log = self.deactivate_into_normal_stop(proc_output, RELEASE_MS)
        future = self.request_active()
        self.activate_with_simulator(log, future)
        powered = log.wait_for_match(DRIVES_POWERED_LINE, timeout=1)
        self.assertIsNotNone(powered, "drives not powered")
        self.assertGreaterEqual(int(powered.group(1)), RELEASE_MS)
        self.assertLess(int(powered.group(1)), DRIVES_ON_TIMEOUT_S * 1000)
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)
        self.assert_latched(set())

        # A release slower than the attempt fails it; the next attempt still powers the drives.
        log = self.deactivate_into_normal_stop(proc_output, SLOW_RELEASE_MS)
        self.assertFalse(self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activated")
        failed = log.wait_for_match(DRIVES_FAILED_LINE, timeout=1)
        self.assertIsNotNone(failed, "no drives-on failure line")
        self.assertGreater(int(failed.group(1)), 1, "DRIVES_ON was not pulsed")
        self.assertTrue(
            self._spin_until(lambda: self.state_value("guard_stop") == 0.0, timeout=10),
            "normal stop never released",
        )
        log = LogCursor(proc_output)
        future = self.request_active()
        self.activate_with_simulator(log, future, respawned=True)
        self.assertIsNotNone(log.wait_for_match(DRIVES_POWERED_LINE, timeout=1), "not powered")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)

        log = LogCursor(proc_output)
        self.inject("normal_stop=0")
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for(STOP_COMPLETE_LINE, timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)

    def test_drives_on_waits_through_peri_flash(self, proc_output):
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()
        self.inject(f"normal_stop=0 operation_mode={OP_MODE_EXT} motion_possible=1")
        log = LogCursor(proc_output)
        future = self.request_active()
        self.activate_with_simulator(log, future, respawned=True)

        # The first request flashes drives_powered while guard_stop still reads 1; starting RSI on
        # that flash would leave the KRC waiting for drives that never come.
        self.inject("transient_peri=1")
        log = self.deactivate_into_normal_stop(proc_output, RELEASE_MS)
        future = self.request_active()
        self.activate_with_simulator(log, future)
        self.assertTrue(log.wait_for(PERI_FLASH_LINE, timeout=1), "no PERI_RDY flash")
        powered = log.wait_for_match(DRIVES_POWERED_LINE, timeout=1)
        self.assertIsNotNone(powered, "drives not powered")
        self.assertGreaterEqual(int(powered.group(1)), RELEASE_MS, "started on the flash")
        self._spin_for(1.0)
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)
        self.assert_latched(set())

        log = LogCursor(proc_output)
        self.inject("normal_stop=0 transient_peri=0")
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for(STOP_COMPLETE_LINE, timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
