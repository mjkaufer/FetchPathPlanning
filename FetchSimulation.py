#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 13 Dec 2018 at 10:33

@author: Andrei
"""

from __future__ import print_function

from math import sin, cos
import numpy as np

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def cross(a, b):
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


# base, torso, shoulder_pan, shoulder_lift, upperarm_roll, elbow_flex, forearm_roll, wrist_roll, wrist_flex, gripper
fetch_transl = np.array([
    np.array([-0.086875, 0, 0.37743]),  # torso
    np.array([0.119525, 0, 0.34858]),  # shoulder_pan
    np.array([0.117, 0, 0.06]),  # shoulder_lift
    np.array([0.219, 0, 0]),  # upperarm_roll
    np.array([0.1279, 0.0073, 0]),  # elbow_flex
    np.array([0.197, 0, 0]),  # forearm_roll
    np.array([0.1245, 0, 0]),  # wrist_flex
    np.array([0.1385, 0, 0]),  # wrist_roll
    np.array([0.16645, 0, 0]),  # gripper
])

# when the robot is at rest!!!
fetch_points_on_axes = np.array([
    np.sum(fetch_transl[:1], axis=0),  # torso
    np.sum(fetch_transl[:2], axis=0),  # shoulder_pan
    np.sum(fetch_transl[:3], axis=0),  # shoulder_lift
    np.sum(fetch_transl[:4], axis=0),  # upperarm_roll
    np.sum(fetch_transl[:5], axis=0),  # elbow_flex
    np.sum(fetch_transl[:6], axis=0),  # forearm_roll
    np.sum(fetch_transl[:7], axis=0),  # wrist_flex
    np.sum(fetch_transl[:8], axis=0),  # wrist_roll
    np.sum(fetch_transl[:9], axis=0),  # tool!
])

fetch_axes = np.array([
    np.array([0, 0, 1]),  # torso
    np.array([0, 0, 1]),  # shoulder_pan
    np.array([0, 1, 0]),  # shoulder_lift
    np.array([1, 0, 0]),  # upperarm_roll
    np.array([0, 1, 0]),  # elbow_flex
    np.array([1, 0, 0]),  # forearm_roll
    np.array([0, 1, 0]),  # wrist_flex
    np.array([1, 0, 0]),  # wrist_roll
])

fetch_twists = np.array([
    np.array([0, 0, 0, 0, 0, 1]),  # torso
    np.concatenate([fetch_axes[1], cross(fetch_points_on_axes[1], fetch_axes[1])], axis=0),  # shoulder_pan
    np.concatenate([fetch_axes[2], cross(fetch_points_on_axes[2], fetch_axes[2])], axis=0),  # shoulder_lift
    np.concatenate([fetch_axes[3], cross(fetch_points_on_axes[3], fetch_axes[3])], axis=0),  # upperarm_roll
    np.concatenate([fetch_axes[4], cross(fetch_points_on_axes[4], fetch_axes[4])], axis=0),  # elbow_flex
    np.concatenate([fetch_axes[5], cross(fetch_points_on_axes[5], fetch_axes[5])], axis=0),  # forearm_roll
    np.concatenate([fetch_axes[6], cross(fetch_points_on_axes[6], fetch_axes[6])], axis=0),  # wrist_flex
    np.concatenate([fetch_axes[7], cross(fetch_points_on_axes[7], fetch_axes[7])], axis=0),  # wrist_roll
])

fetch_joint_limits = [
    [0, 0.38615],  # torso
    [-1.6056, 1.6056],  # shoulder_pan
    [-1.221, 1.518],  # shoulder_lift
    [0, 2 * np.pi],  # upperarm_roll
    [-2.251, 2.251],  # elbow_flex
    [0, 2 * np.pi],  # forearm_roll
    [-2.16, 2.16],  # wrist_flex
    [0, 2 * np.pi],  # wrist_roll
]


# Move base using navigation stack
class MoveBaseClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def go_to(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta / 2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta / 2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()


# Send a trajectory to controller
class FollowTrajectoryClient(object):
    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(int(duration))
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()


class FetchSimulation:
    class RobotJoint:
        def __init__(self, name, min_range=None, max_range=None, max_velocity=None, max_force=None):
            self.name = name  # type: str
            self.position = 0.0  # type: float
            self.velocity = 0.0  # type: float
            self.force = 0.0  # type: float

            if min_range is None or max_range is None:
                self.joint_range = None
            else:
                self.joint_range = (min_range, max_range)

            if max_velocity is None:
                self.velocity_range = None
            else:
                self.velocity_range = (0.0, max_velocity)

            if max_force is None:
                self.force_range = None
            else:
                self.force_range = (0.0, max_force)

        def update(self, position, velocity, force):
            self.position = position
            self.velocity = velocity
            self.force = force

    def __init__(self):
        print("Make sure you have launched \n\t\"roslaunch fetch_moveit_config move_group.launch\"\n"
              "before instantiating the robot simulation!")

        self.l_wheel_joint = FetchSimulation.RobotJoint("l_wheel_joint", max_velocity=17.4, max_force=8.85)
        self.r_wheel_joint = FetchSimulation.RobotJoint("r_wheel_joint", max_velocity=17.4, max_force=8.85)
        self.base_joints = [self.l_wheel_joint, self.r_wheel_joint]

        self.torso_joint = FetchSimulation.RobotJoint("torso_lift_joint", min_range=0, max_range=400, max_velocity=0.1,
                                                      max_force=450)
        self.torso_joints = [self.torso_joint]

        self.head_pan_joint = FetchSimulation.RobotJoint("head_pan_joint", min_range=-90, max_range=90,
                                                         max_velocity=1.57, max_force=0.32)
        self.head_tilt_joint = FetchSimulation.RobotJoint("head_tilt_joint", min_range=-45, max_range=90,
                                                          max_velocity=1.57,
                                                          max_force=0.68)  # -45 upwards and 90 downwards
        self.head_joints = [self.head_pan_joint, self.head_tilt_joint]

        self.shoulder_pan_joint = FetchSimulation.RobotJoint("shoulder_pan_joint", min_range=-92, max_range=92,
                                                             max_velocity=1.25, max_force=33.82)
        self.shoulder_lift_joint = FetchSimulation.RobotJoint("shoulder_lift_joint", min_range=-70, max_range=87,
                                                              max_velocity=1.45, max_force=131.76)
        self.upperarm_roll_joint = FetchSimulation.RobotJoint("upperarm_roll_joint", max_velocity=1.57, max_force=76.94)
        self.elbow_flex_joint = FetchSimulation.RobotJoint("elbow_flex_joint", min_range=-129, max_range=129,
                                                           max_velocity=1.52, max_force=66.18)
        self.forearm_roll_joint = FetchSimulation.RobotJoint("forearm_roll_joint", max_velocity=1.57, max_force=29.35)
        self.wrist_flex_joint = FetchSimulation.RobotJoint("wrist_flex_joint", min_range=-125, max_range=125,
                                                           max_velocity=2.26, max_force=25.7)
        self.wrist_roll_joint = FetchSimulation.RobotJoint("wrist_roll_joint", max_velocity=2.26, max_force=7.36)
        self.arm_joints = [self.shoulder_pan_joint, self.shoulder_lift_joint, self.upperarm_roll_joint,
                           self.elbow_flex_joint, self.forearm_roll_joint, self.wrist_flex_joint, self.wrist_roll_joint]

        self.l_gripper_finger_joint = FetchSimulation.RobotJoint("l_gripper_finger_joint", min_range=0, max_range=50,
                                                                 max_velocity=0.05, max_force=60)
        self.r_gripper_finger_joint = FetchSimulation.RobotJoint("r_gripper_finger_joint", min_range=0, max_range=50,
                                                                 max_velocity=0.05, max_force=60)
        self.gripper_joints = [self.l_gripper_finger_joint, self.r_gripper_finger_joint]

        self.joints = self.base_joints + self.torso_joints + self.head_joints + self.arm_joints + self.gripper_joints
        assert (len(self.joints) == 14)

        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.orientation = 0.0

        self.base_joint_names = [x.name for x in self.base_joints]
        self.torso_joint_names = [x.name for x in self.torso_joints]
        self.head_joint_names = [x.name for x in self.head_joints]
        self.arm_joint_names = [x.name for x in self.arm_joints]
        self.gripper_joint_names = [x.name for x in self.gripper_joints]
        self.joint_names = [x.name for x in self.joints]

        # Create move group interface for a fetch robot
        self.move_group = MoveGroupInterface("arm_with_torso", "base_link")

        # Define ground plane
        # This creates objects in the planning scene that mimic the ground
        # If these were not in place gripper could hit the ground
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene.removeCollisionObject("my_front_ground")
        self.planning_scene.removeCollisionObject("my_back_ground")
        self.planning_scene.removeCollisionObject("my_right_ground")
        self.planning_scene.removeCollisionObject("my_left_ground")
        self.planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
        self.planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
        self.planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        self.planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    def move_to_pose(self, pose_10d, relative=False):
        arm_joint_values = pose_10d[:7]
        base_joint_values = pose_10d[7:]
        assert (len(arm_joint_values) == 7)
        assert (len(base_joint_values) == 3)

        # for moving the base
        move_base = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # for moving the arm and the torso
        move_arm_and_torso = actionlib.SimpleActionClient("arm_with_torso_controller/follow_joint_trajectory",
                                                          FollowJointTrajectoryAction)

        try:
            base_goal = Twist()
            if relative:
                base_goal.linear.x += base_joint_values[0]  # x
                # base_goal.linear.y = base_joint_values[1]  # y
                base_goal.angular.z += base_joint_values[2]  # theta
            else:
                base_goal.linear.x = base_joint_values[0]  # x
                # base_goal.linear.y = base_joint_values[1]  # y
                base_goal.angular.z = base_joint_values[2]  # theta
            move_base.publish(base_goal)

            if relative:
                self.shoulder_pan_joint.position += arm_joint_values[0]  # arm1
                self.shoulder_lift_joint.position += arm_joint_values[1]  # arm2
                self.upperarm_roll_joint.position += arm_joint_values[2]  # arm3
                self.elbow_flex_joint.position += arm_joint_values[3]  # arm4
                self.forearm_roll_joint.position += arm_joint_values[4]  # arm5
                self.wrist_flex_joint.position += arm_joint_values[5]  # arm6
                self.wrist_roll_joint.position += arm_joint_values[6]  # arm7
            else:
                self.shoulder_pan_joint.position = arm_joint_values[0]  # arm1
                self.shoulder_lift_joint.position = arm_joint_values[1]  # arm2
                self.upperarm_roll_joint.position = arm_joint_values[2]  # arm3
                self.elbow_flex_joint.position = arm_joint_values[3]  # arm4
                self.forearm_roll_joint.position = arm_joint_values[4]  # arm5
                self.wrist_flex_joint.position = arm_joint_values[5]  # arm6
                self.wrist_roll_joint.position = arm_joint_values[6]  # arm7
            arm_and_torso_desired_position = [x.position for x in self.torso_joints + self.arm_joints]

            arm_and_torso_trajectory = JointTrajectory()
            arm_and_torso_trajectory.joint_names = self.torso_joint_names + self.arm_joint_names
            arm_and_torso_trajectory.points.append(JointTrajectoryPoint())
            arm_and_torso_trajectory.points[0].positions = arm_and_torso_desired_position
            arm_and_torso_trajectory.points[0].velocities = [0.0 for _ in arm_and_torso_desired_position]
            arm_and_torso_trajectory.points[0].accelerations = [0.0 for _ in arm_and_torso_desired_position]
            arm_and_torso_trajectory.points[0].time_from_start = rospy.Duration(int(5))

            arm_and_torso_goal = FollowJointTrajectoryGoal()
            arm_and_torso_goal.trajectory = arm_and_torso_trajectory

            move_arm_and_torso.send_goal_and_wait(arm_and_torso_goal)
        except Exception as e:
            print(e)
            move_base.publish(Twist())  # stop the base from moving
            move_arm_and_torso.cancel_all_goals()


if __name__ == "__main__":
    for i in fetch_points_on_axes:
        print(i)
