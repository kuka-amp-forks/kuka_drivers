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

"""A repeat count still running when deactivated is logged before ResetDiagnostics clears it."""

import os
import re
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    SIMULATOR_SILENCE_PACKETS,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

FIRST_REPEAT_LINE = re.compile(r"Repeated IPOC (\d+): the controller sent the same cycle again")
RUN_LINE = re.compile(r"IPOC (\d+) repeated (\d+) time\(s\)")


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description()


class TestLogsRepeatedIpocOnDeactivate(RsiFaultTestCase):
    def test_logs_repeated_ipoc_on_deactivate(self, proc_output):
        log = LogCursor(proc_output)
        self.start_streaming(log)

        # A repeat count still running when deactivated never sees a new IPOC to log it against.
        log = LogCursor(proc_output)
        self.set_simulator_fault("fault_repeated_ipoc_count", SIMULATOR_SILENCE_PACKETS)
        first = log.wait_for_match(FIRST_REPEAT_LINE, timeout=5)
        self.assertIsNotNone(first, "no repeated-IPOC warning")
        ipoc = int(first.group(1))
        self.assertTrue(
            self._spin_until(lambda: self.state_ipocs.count(ipoc) >= 3, timeout=5),
            f"IPOC {ipoc} not repeated a few times: {self.state_ipocs}",
        )

        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        run = log.wait_for_match(RUN_LINE, timeout=5)
        self.assertIsNotNone(run, "repeated-IPOC count not logged on deactivate")
        self.assertEqual(int(run.group(1)), ipoc)
        self.assertGreaterEqual(int(run.group(2)), 3)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
