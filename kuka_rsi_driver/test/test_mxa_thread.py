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

"""The mxA thread runs with the control loop's scheduling."""

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
    EVENT_BROADCASTER,
    INHIBITOR_BROADCASTER,
    JSB,
    RsiFaultTestCase,
    fault_launch_description,
)

# The control loop's core; any core the container has.
LOOP_CPU = 1
FAKE_THREAD_NAME = "mxa_status_fake"
SCHED_FIFO = 1
LOOP_PRIORITY_LINE = re.compile(r"Control loop priority was set to (\d+)")
LOOP_NOT_FIFO_LINE = "Control loop is not SCHED_FIFO, the mxA thread keeps the default scheduling"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(
        respawn_simulator=True, controller_manager_params={"cpu_affinity": LOOP_CPU}
    )


def fake_thread(pid):
    """(Cpus_allowed_list, policy, rt_priority) of the fake's receive thread in pid."""
    task_dir = f"/proc/{pid}/task"
    for tid in os.listdir(task_dir):
        try:
            with open(f"{task_dir}/{tid}/comm") as f:
                if f.read().strip() != FAKE_THREAD_NAME:
                    continue
            with open(f"{task_dir}/{tid}/status") as f:
                cpus = re.search(r"Cpus_allowed_list:\s*(\S+)", f.read()).group(1)
            with open(f"{task_dir}/{tid}/stat") as f:
                fields = f.read().rsplit(")", 1)[1].split()
        except FileNotFoundError:
            continue
        # stat fields 40 and 41, counted from field 3 after the command name.
        return cpus, int(fields[38]), int(fields[37])
    return None


class TestMxaThread(RsiFaultTestCase):
    def prepare_inactive(self):
        """INACTIVE with read() running and the broadcasters active, also after an mxA loss."""
        if self.component_state() == State.PRIMARY_STATE_UNCONFIGURED:
            self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "reconfigure")
            self.switch(activate=[JSB, INHIBITOR_BROADCASTER, EVENT_BROADCASTER])
        for name in (JSB, INHIBITOR_BROADCASTER, EVENT_BROADCASTER):
            self._wait_for_controller_active(name, timeout=30)
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()

    def test_copies_loop_scheduling(self, proc_output, proc_info):
        self.prepare_inactive()
        self._spin_for(0.5)
        thread = fake_thread(proc_info["control_node"].pid)
        self.assertIsNotNone(thread, "no mxA status fake thread in control_node")
        cpus, policy, priority = thread
        self.assertEqual(cpus, str(LOOP_CPU))

        output = "".join(event.text.decode(errors="replace") for event in proc_output)
        loop_priority = LOOP_PRIORITY_LINE.search(output)
        if loop_priority is None:
            # Not permitted here, as in a container without CAP_SYS_NICE: logged, not fatal.
            self.assertTrue(LOOP_NOT_FIFO_LINE in output, "no fail-soft warning")
            self.assertNotEqual(policy, SCHED_FIFO)
        else:
            self.assertEqual(policy, SCHED_FIFO)
            self.assertEqual(priority, int(loop_priority.group(1)) - 1)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
