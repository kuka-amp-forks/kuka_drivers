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

"""Shared base for launch tests that move a joint through joint_trajectory_controller."""

import time
import unittest

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [f"joint_{i}" for i in range(1, 7)]
MOVED_JOINT = "joint_1"
MOVE_RAD = 0.1
TOLERANCE_RAD = 1e-3


class JointMoveTestCase(unittest.TestCase):
    """unittest.TestCase with a ROS node, /joint_states tracking, and service/action helpers."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(f"{self.__class__.__name__.lower()}_test")
        self.positions = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

    def tearDown(self):
        self.node.destroy_node()

    def _on_joint_states(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def _spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _spin_until(self, condition, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if condition():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def _call(self, srv_type, name, request, timeout):
        client = self.node.create_client(srv_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=timeout), name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertIsNotNone(future.result(), f"{name} did not answer")
        return future.result()

    def _wait_future(self, future, timeout):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), "action call timed out")
        return future.result()

    def _wait_for_controller_active(self, controller_name, timeout):
        def active():
            result = self._call(
                ListControllers,
                "/controller_manager/list_controllers",
                ListControllers.Request(),
                timeout=timeout,
            )
            return any(
                c.name == controller_name and c.state == "active" for c in result.controller
            )

        self.assertTrue(
            self._spin_until(active, timeout), f"{controller_name} did not become active"
        )

    def move_joint_and_wait(self, move_timeout=15):
        self.assertTrue(
            self._spin_until(lambda: all(j in self.positions for j in JOINTS), timeout=10),
            f"no joint_states for {JOINTS}",
        )
        start = [self.positions[j] for j in JOINTS]
        target = list(start)
        target[JOINTS.index(MOVED_JOINT)] += MOVE_RAD

        action = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.assertTrue(action.wait_for_server(timeout_sec=20), "trajectory action not available")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=target, time_from_start=Duration(sec=1))
        ]
        handle = self._wait_future(action.send_goal_async(goal), timeout=10)
        self.assertTrue(handle.accepted, "trajectory goal rejected")

        reached = self._spin_until(
            lambda: abs(self.positions[MOVED_JOINT] - target[JOINTS.index(MOVED_JOINT)])
            < TOLERANCE_RAD,
            timeout=move_timeout,
        )
        self.assertTrue(
            reached,
            f"{MOVED_JOINT} at {self.positions[MOVED_JOINT]:.4f}, "
            f"target {target[JOINTS.index(MOVED_JOINT)]:.4f}",
        )

        outcome = self._wait_future(handle.get_result_async(), timeout=15)
        self.assertEqual(outcome.status, GoalStatus.STATUS_SUCCEEDED)
