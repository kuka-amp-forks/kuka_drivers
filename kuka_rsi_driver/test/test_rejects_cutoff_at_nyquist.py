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

"""A velocity_filter_cutoff_hz at or above Nyquist (125 Hz at the 4 ms RSI cycle) fails on_init."""

import os
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    RsiFaultTestCase,
    fault_launch_description,
    hardware_params,
)

NYQUIST_HZ = 125.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(extra_params=hardware_params(velocity_filter_cutoff_hz=125))


class TestRejectsCutoffAtNyquist(RsiFaultTestCase):
    def test_rejects_cutoff_at_nyquist(self, proc_output):
        proc_output.assertWaitFor(
            f"velocity_filter_cutoff_hz ({NYQUIST_HZ:.1f} Hz) must be below the Nyquist "
            f"frequency ({NYQUIST_HZ:.1f} Hz)",
            timeout=10,
        )
        proc_output.assertWaitFor("Failed to initialize hardware", timeout=5)
        # Shutting down before the simulator is up leaves it ignoring SIGINT until SIGTERM.
        proc_output.assertWaitFor("[kuka_rsi_simulator]: Started", timeout=10)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # The simulator never exchanges a packet with the never-initialized hardware, so SIGINT
        # kills it blocked in its receive loop (-2) instead of it exiting on a stop flag.
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK, -2],
        )
