from vpython import *
from Fetch import Fetch
import numpy as np
import time
from Vis import init, rerender
from math import radians as r
from BiRRT import BiRRT

fetch = Fetch()

init(fetch)
goal = np.array([[800,000,100]]).T

def goToGoal(goal):
    print("Finding ideal pose")
    idealPose, error = fetch.inverseKinematics(goal, verbose=0)
    print("Ideal pose found", idealPose, "with error of", error)
    segmentation = 100
    poseDelta = (np.array(idealPose) - np.array(fetch.getPoses())) / segmentation
    for i in range(segmentation):
        fetch.applyRelativePoses(poseDelta)
        rerender(fetch)
        time.sleep(0.05)

def exampleFloorCollision():
    degreePoses = [0, 40, 0, -20, 0, -30, 0]
    poses = [r(degreePose) for degreePose in degreePoses]
    fetch.applyPoses(poses)
    rerender(fetch)

def exampleBodyCollision():
    degreePoses = [0, -60, 0, -120, 0, -60, 0]
    poses = [r(degreePose) for degreePose in degreePoses]
    fetch.applyPoses(poses)
    rerender(fetch)

def goToGoalWithRRT(goal):
    print("Finding ideal pose")
    # idealPose, error = fetch.inverseKinematics(goal, verbose=1)
    # print("Ideal pose found", idealPose, "with error of", error)
    idealPose = np.array([-1.13793837, -0.42068421, 5.03419161, -2.0732122,
        5.94965048, -0.82803775, 5.27579817])

    rrt = BiRRT(fetch, idealPose)
    i = 0
    while rrt.solution is None:
        i += 1
        rrt.step()
        if i % 100 == 0:
            print(i)
            print("\tStart Tree Length",len(rrt.startTree))
            print("\tGoal Tree Length",len(rrt.goalTree))
            print(rrt.solution)

    # desiredTime = 5.0
    # dt = desiredTime / len(rrt.solution)
    print("Planning complete; moving to goal pose")

    for pose in rrt.solution:
        fetch.applyPoses(pose)
        segmentPositions = fetch.getSegmentPositions()

        if not fetch.isCurrentPoseValid():
            print("INVALID POSE BEEP BOOP")

        rerender(fetch)

    print("All done")
    print("Smallest segment position was", minZ)
    print("Start tree length", len(rrt.startTree))
    print("Goal tree length", len(rrt.goalTree))
    print("Solution length", len(rrt.solution))

    solutionRatio = round(len(rrt.solution) / (len(rrt.startTree) + len(rrt.goalTree)) * 100, 2)
    print(solutionRatio,"% of RRT used for solution")
    

# exampleBodyCollision()
goToGoalWithRRT(goal)
print("Is the pose valid?",fetch.isPoseValid())