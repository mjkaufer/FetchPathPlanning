#!/usr/bin/env python

# from vpython import *
import rospy
from Fetch import Fetch
import numpy as np
import time
# from Vis import init, rerender
from math import radians as r
from BiRRT import BiRRT
from FetchSimulation import FetchSimulation


def example_floor_collision(robot, robot_sim):
    degreePoses = [0, 40, 0, -20, 0, -30, 0]
    poses = [r(degreePose) for degreePose in degreePoses]
    robot.applyPoses(poses)
    # rerender(robot)
    robot_sim.move_to_pose(poses, relative=False)


def example_body_collision(robot, robot_sim):
    degreePoses = [0, -60, 0, -120, 0, -60, 0]
    poses = [r(degreePose) for degreePose in degreePoses]
    robot.applyPoses(poses)
    # rerender(robot)
    robot_sim.move_to_pose(poses, relative=False)


def go_to_goal(robot, robot_sim, goal):
    # sphere(pos=vector(*goal.flatten()), radius=25, color=vector(1, 0, 0))
    print("Finding ideal pose")
    idealPose, error = robot.inverseKinematics(goal, verbose=0)
    print("Ideal pose found", idealPose, "with error of", error)
    go_to_pose(robot, robot_sim, idealPose)


def go_to_pose(robot, robot_sim, goal_pose):
    segmentation = 100
    pose_delta = (np.array(goal_pose) - np.array(robot.getPoses())) / segmentation
    for i in range(segmentation):
        robot.applyRelativePoses(pose_delta)

        if not robot.isPoseValid():
            print("Invalid pose!!!")

        # rerender(robot)
        time.sleep(0.05)
        print("Moving simulation robot")
        robot_sim.move_to_pose(pose_delta, relative=True)
        print("Moved simulation robot")


def go_to_goal_with_rrt(robot, robot_sim, goal):
    # sphere(pos=vector(*goal.flatten()), radius=25, color=vector(1, 0, 0))
    print("Finding ideal pose")
    ideal_pose, error = robot.inverseKinematics(goal, verbose=1)
    # print("Ideal pose found", ideal_pose, "with error of", error)
    # ideal_pose = np.array([-1.13793837, -0.42068421, 5.03419161, -2.0732122,
    #     5.94965048, -0.82803775, 5.27579817])
    go_to_pose_with_rrt(robot, robot_sim, ideal_pose)


def go_to_pose_with_rrt(robot, robot_sim, goal_pose):
    rrt = BiRRT(robot, goal_pose)
    i = 0
    print("Generating RRT")
    while rrt.solution is None:
        i += 1
        rrt.step()
        if i % 100 == 0:
            print(i)
            print("\tStart Tree Length", len(rrt.startTree))
            print("\tGoal Tree Length", len(rrt.goalTree))
            print(rrt.solution)

    # desiredTime = 5.0
    # dt = desiredTime / len(rrt.solution)
    print("Planning complete; moving to goal pose")

    for goal_pose in rrt.solution:
        robot.applyPoses(goal_pose)
        if not robot.isCurrentPoseValid():
            print("INVALID POSE BEEP BOOP")

        # rerender(robot)
        robot_sim.move_to_pose(goal_pose, relative=False)

    print("All done")
    print("Start tree length", len(rrt.startTree))
    print("Goal tree length", len(rrt.goalTree))
    print("Solution length", len(rrt.solution))

    solutionRatio = round(len(rrt.solution) / (len(rrt.startTree) + len(rrt.goalTree)) * 100, 2)
    print(solutionRatio, "% of RRT used for solution")


if __name__ == "__main__":
    rospy.init_node(name="FetchRRT")
    # [-0.43352042 -1.43271184  6.04244123 -1.75627955  3.6502145   2.1716629 2.67857958]
    fetch = Fetch()
    fetch_sim = FetchSimulation()

    # init(fetch)
    # goal = np.array([[500, 000, -50]]).T
    print("Defined the goal position")

    # here is an example of a pose that was found for an end goal, which has
    # a linear path planning that would result in an invalid pose

    pose = np.array(
        [-1, 0.2, 7.71942311e-01, 2.24147991e+00, 4.61934400e-04, 1.03087359e+00, 2.18929696e-03, 1, 1, np.pi / 2])
    go_to_pose(fetch, fetch_sim, pose)
    print("Went to the pose...")

    # and here is how we get to the pose with RRT
    go_to_pose_with_rrt(fetch, fetch_sim, pose)
    print("Is the pose valid?", fetch.isPoseValid())

    # example_body_collision(fetch)
    # go_to_goal_with_rrt(fetch, goal)
    # go_to_goal(fetch, goal)

    # go_to_pose(fetch)
    # rerender(fetch)
