from Segment import Segment
from math import radians as r
from util import extractTranslation
import numpy as np

defaultArmLength = 200 # mm; this is a guess, we can change it later
# the axes also might be off – todo, actually verify this
# defaultFetchJoints = [
#     (defaultArmLength, 'z'),
#     (defaultArmLength, 'y'),
# ]

class Fetch:
    # joints is an array in the form [...(jointLength, jointAxis, smallJointLimit, largeJointLimit)...]
    # joints at the front of the array are joints closest to the robot
    # define x to be axis pointing from fetch's face, y to be axis perpendicular to x axis and parallel to ground,
    # and z axis to be pointing up
    # let's just say units are in millimeters
    def __init__(self):

        self.segments = [
            # shoulder pan joint
            Segment(defaultArmLength, 'z', r(-92), r(92)),

            # shoulder lift joint
            Segment(defaultArmLength, 'y', r(-87), r(70)),

            # upper arm roll joint – default limits of 0 - 360
            Segment(defaultArmLength, 'x'),

            # elbow flex joint
            Segment(defaultArmLength, 'y', r(-129), r(129)),

            # forearm roll joint – default limits of 0 - 360
            Segment(defaultArmLength, 'x', ),

            # wrist flex joint
            Segment(defaultArmLength, 'y', r(-125), r(125)),

            # wrist roll joint – default limits of 0 - 360
            Segment(defaultArmLength, 'x', ),
        ]

        for i in range(len(self.segments)):
            if i > 0:
                self.segments[i].parentSegment = self.segments[i - 1]

    def getPoses(self):
        return [segment.currentRotation for segment in self.segments]

    def setRandomPoses(self):
        for segment in self.segments:
            rotation = segment.randomRotation()
            segment.rotate(rotation)

    # poseArray is an array of radians
    def applyPoses(self, poseArray):
        for i in range(len(self.segments)):
            self.segments[i].rotate(poseArray[i])

    def applyRelativePoses(self, relativePoseArray):
        for i in range(len(self.segments)):
            self.segments[i].relativeRotate(relativePoseArray[i])

    def getSegmentPosition(self, segmentIndex):
        return extractTranslation(self.segments[segmentIndex].computeGlobalTransformationMatrix())

    def getTool(self):
        return self.getSegmentPosition(-1)

    # destination is a 3x1 xyz
    def getPoseDeltas(self, destination, delta=1e-2):

        initialToolPos = self.getTool()
        currentError = np.vstack([destination - initialToolPos, 1])

        jac = []

        for i in range(len(self.segments)):
            self.segments[i].relativeRotate(delta)
            currentToolPos = self.getTool()
            self.segments[i].relativeRotate(-delta)
            toolDelta = np.vstack([currentToolPos - initialToolPos, 1])
            jac.append(toolDelta)

        jac = np.array(jac)[:,:,0]

        return jac.dot(currentError)

    # goal is a 3x1 np array
    # if verbose is 0, shh
    # if verbose is 1, just print batch stats
    # if verbose is 2, print errors every 5 samples

    def inverseKinematics(self, goal, batchSize=1000, errorThresh=5, verbose=1):

        currentError = 10e10

        initialPoses = fetch.getPoses()

        while currentError > errorThresh:
            if verbose > 0:
                print("SGD Batch")
            fetch.setRandomPoses()

            for i in range(batchSize):
                deltas = fetch.getPoseDeltas(goal)
                currentError = np.linalg.norm(goal - fetch.getTool())

                # scalar represents the amount of the normalized delta vector we want to apply
                scalar = 10 ** (-1 * max(5 - np.log(currentError), 1))
                scaledDeltas = (deltas / np.linalg.norm(deltas)) * scalar
                scaledDeltas = np.array(scaledDeltas.tolist()).flatten()
                fetch.applyRelativePoses(scaledDeltas)

                if i % 5 == 0 and verbose > 1:
                    print(currentError)

                if currentError < errorThresh:
                    break

            if verbose > 0 and currentError > errorThresh:
                print("This batch resulted in an error of", currentError, "- rerunning")

        goalPoses = fetch.getPoses()
        fetch.applyPoses(initialPoses)

        if verbose > 0:
            print("Finished with an error of", currentError)

        return goalPoses, currentError

fetch = Fetch()
goal = np.array([[200, 0, 0]]).T
goalPoses, goalError = fetch.inverseKinematics(goal)
print("Reached goal with pose", goalPoses, "with error of", goalError)