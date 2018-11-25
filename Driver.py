from vpython import *
from Fetch import Fetch
import numpy as np
import time
from Vis import init, rerender
from math import radians as r

fetch = Fetch()

init(fetch)
goal = np.array([[200,100,100]]).T

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

def exampleCollision():
    degreePoses = [0, 20, 0, -129, 0, -120, 0]
    poses = [r(degreePose) for degreePose in degreePoses]
    fetch.applyPoses(poses)
    rerender(fetch)

    print(fetch.isPoseValid())

exampleCollision()
# goToGoal(goal)