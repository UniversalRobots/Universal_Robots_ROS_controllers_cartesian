#!/usr/bin/env python
import sys
import unittest
import copy
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import time
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    FollowCartesianTrajectoryResult,
    CartesianTrajectoryPoint)
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController

PKG = 'cartesian_trajectory_controller'
NAME = 'test_controller'


class IntegrationTest(unittest.TestCase):
    def __init__(self, *args):
        super(IntegrationTest, self).__init__(*args)

        rospy.init_node('test_controller')

        timeout = rospy.Duration(3)

        self.set_joints = actionlib.SimpleActionClient(
            "/joint_trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        try:
            self.set_joints.wait_for_server(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail("Could not reach controller action. Msg: {}".format(err))

        self.cart_client = actionlib.SimpleActionClient(
            'cartesian_trajectory_controller/follow_cartesian_trajectory', FollowCartesianTrajectoryAction)
        try:
            self.cart_client.wait_for_server(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail("Could not reach controller action. Msg: {}".format(err))

        self.pub_trajectory = actionlib.SimpleActionClient(
            'cartesian_trajectory_publisher/follow_cartesian_trajectory', FollowCartesianTrajectoryAction)
        if not self.cart_client.wait_for_server(timeout):
            self.fail("Could not reach trajectory publisher action.")

        self.switch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        try:
            self.switch.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach controller switch service. Msg: {}".format(err))

    def test_normal_execution(self):
        """ Test the basic functionality on a straight line """
        self.move_to_start()
        self.switch_to_cartesian_control()
        self.move_square(self.cart_client)
        self.assertEqual(self.cart_client.get_result().error_code,
                         FollowCartesianTrajectoryResult.SUCCESSFUL)

    def test_preemption(self):
        """ Test whether the preemption mechanism works correctly """
        self.move_to_start()
        self.switch_to_cartesian_control()
        self.move_square(self.cart_client, wait_for_result=False)
        time.sleep(3)  # stop somewhere during execution
        self.cart_client.cancel_goal()
        time.sleep(1)
        self.assertEqual(self.cart_client.get_state(),
                         GoalStatus.PREEMPTED)

    def test_invalid_goals(self):
        """ Test whether invalid goals are rejected correctly """

        self.move_to_start()
        self.switch_to_cartesian_control()

        # Time from start is not strictly increasing.
        # This also covers points without time_from_start (they are all implicitly 0).
        p1 = CartesianTrajectoryPoint()
        p1.time_from_start = rospy.Duration(1.0)
        p2 = CartesianTrajectoryPoint()
        p2.time_from_start = rospy.Duration(0.9999)
        goal = FollowCartesianTrajectoryGoal()
        goal.trajectory.points.append(p1)
        goal.trajectory.points.append(p2)
        self.cart_client.send_goal(goal)
        self.cart_client.wait_for_result()
        self.assertEqual(self.cart_client.get_result().error_code,
                         FollowCartesianTrajectoryResult.INVALID_GOAL)

    def test_trajectory_publishing(self):
        """ Test whether trajectory publishing works """

        self.move_to_start()

        # Activate trajectory publishing
        srv = SwitchControllerRequest()
        srv.stop_controllers = []
        srv.start_controllers = ['cartesian_trajectory_publisher']
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch(srv)

        self.move_square(self.pub_trajectory)

        # Deactivate trajectory publishing to not spam the other tests.
        srv = SwitchControllerRequest()
        srv.stop_controllers = ['cartesian_trajectory_publisher']
        srv.start_controllers = []
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch(srv)

        self.assertEqual(self.pub_trajectory.get_result().error_code,
                         FollowCartesianTrajectoryResult.SUCCESSFUL)

    def move_to_start(self):
        srv = SwitchControllerRequest()
        srv.stop_controllers = ['cartesian_trajectory_controller']
        srv.start_controllers = ['joint_trajectory_controller']
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch(srv)

        q_start = [0, -2.0, 2.26, -0.2513274122872, 1.57, 0.0]  # From base to tip
        start_joint_state = FollowJointTrajectoryGoal()
        start_joint_state.trajectory.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6']

        start_joint_state.trajectory.points = [
            JointTrajectoryPoint(positions=q_start, time_from_start=rospy.Duration(3.0))]
        self.set_joints.send_goal(
            start_joint_state)
        self.set_joints.wait_for_result()

    def switch_to_cartesian_control(self):
        srv = SwitchControllerRequest()
        srv.stop_controllers = ['joint_trajectory_controller']
        srv.start_controllers = ['cartesian_trajectory_controller']
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch(srv)

    def move_square(self, action_client, wait_for_result=True):
        """ Follow a 20 cm square in the y-z plane within 10 sec """

        # Cartesian end-effector pose that corresponds to the start joint
        # configuration
        start = CartesianTrajectoryPoint()
        start.pose.position.x = 0.354
        start.pose.position.y = 0.180
        start.pose.position.z = 0.390
        start.pose.orientation.x = 0.502
        start.pose.orientation.y = 0.502
        start.pose.orientation.z = 0.498
        start.pose.orientation.w = 0.498

        duration = 10

        p1 = copy.deepcopy(start)
        p1.pose.position.y += 0.2
        p1.time_from_start = rospy.Duration(1.0 / 4 * duration)

        p2 = copy.deepcopy(p1)
        p2.pose.position.z += 0.2
        p2.time_from_start = rospy.Duration(2.0 / 4 * duration)

        p3 = copy.deepcopy(p2)
        p3.pose.position.y -= 0.2
        p3.time_from_start = rospy.Duration(3.0 / 4 * duration)

        p4 = copy.deepcopy(p3)
        p4.pose.position.z -= 0.2
        p4.time_from_start = rospy.Duration(4.0 / 4 * duration)

        goal = FollowCartesianTrajectoryGoal()
        goal.trajectory.points.append(p1)
        goal.trajectory.points.append(p2)
        goal.trajectory.points.append(p3)
        goal.trajectory.points.append(p4)

        action_client.send_goal(goal)
        if wait_for_result:
            action_client.wait_for_result()


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, IntegrationTest, sys.argv)
