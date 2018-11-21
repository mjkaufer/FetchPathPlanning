import numpy as np
from util import axisToRoll, buildRotationMatrix, matrixBottom

class Segment:
    def __init__(self, segmentLength, segmentAxis, smallestJointLimit=0, largestJointLimit=2*np.pi, parentSegment=None):
        # self.translation = np.roll(np.array([segmentLength, 0, 0]), axisToRoll(segmentAxis))
        self.translation = np.matrix([segmentLength, 0, 0]).T
        self.segmentAxis = segmentAxis
        self.currentRotation = 0
        self.parentSegment = parentSegment
        self.smallestJointLimit = smallestJointLimit
        self.largestJointLimit = largestJointLimit
        self.transformationMatrix = self.computeTransformationMatrix(self.currentRotation)

    def clamp(self, angle):
        return max(min(angle, self.largestJointLimit), self.smallestJointLimit)
    def rotate(self, rotationInRadians):
        self.currentRotation = self.clamp(rotationInRadians)

    def relativeRotate(self, delta):
        self.currentRotation = self.clamp(delta + self.currentRotation)

    def randomRotation(self):
        return np.random.uniform(self.smallestJointLimit, self.largestJointLimit)

    def computeTransformationMatrix(self, deltaRotationInRadians=0):
        rotationInRadians = self.clamp(self.currentRotation + deltaRotationInRadians)

        # if rotationInRadians < self.smallestJointLimit:
        #     raise ValueError("Your rotation amount is smaller than the joint limit!")
        # if rotationInRadians > self.largestJointLimit:
        #     raise ValueError("Your rotation amount is larger than the joint limit!")

        rotationMatrix = buildRotationMatrix(self.segmentAxis, rotationInRadians)
        matrixTop = np.hstack([rotationMatrix, self.translation])
        return np.vstack([matrixTop, matrixBottom])

    def computeGlobalTransformationMatrix(self):
        if self.parentSegment is None:
            return self.computeTransformationMatrix()

        parentMatrix = self.parentSegment.computeGlobalTransformationMatrix()

        return np.dot(parentMatrix, self.computeTransformationMatrix())