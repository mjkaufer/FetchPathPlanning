from Fetch import Fetch
import numpy as np

fetch = Fetch()
goal = np.array([[200, 0, 0]]).T
goalPoses, goalError = fetch.inverseKinematics(goal)
print("Reached goal with pose", goalPoses, "with error of", goalError)