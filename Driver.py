from vpython import *
from Fetch import Fetch
import numpy as np
import time
from Vis import init, rerender

fetch = Fetch()

init(fetch)
goal = np.array([[200,100,100]]).T

idealPose, error = fetch.inverseKinematics(goal, verbose=0)
print("Ideal pose found", idealPose, "with error of", error)
segmentation = 100
poseDelta = (np.array(idealPose) - np.array(fetch.getPoses())) / segmentation
for i in range(segmentation):
    fetch.applyRelativePoses(poseDelta)
    rerender(fetch)
    time.sleep(0.05)