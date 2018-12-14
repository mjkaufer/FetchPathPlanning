#!/usr/bin/env python
# -*- coding: utf-8 -*-

from Segment import Segment
from math import radians as r
from FetchUtil import extractTranslation, closestDistanceBetweenLines
import numpy as np
# from vpython import vector, cylinder

# base diameter: 23 in
# base height: 14.25 in
# head radius: 9.5 in
# arms max.width: 6 in

defaultArmLength = 219  # mm; this is a guess, we can change it later
armThreshold = 25  # this is how close a joint can be to a base element
# it should basically represent the thickness of a joint
# so the thickness of the arm joints is 6 inches ~= 15.24cm -> TODO: armThreshold = 0.153 meters

# nb, the z axis decreases as you go higher, but that's fine

topBaseHeight = 500
topBaseRadius = 100
topBasePosition = np.array([defaultArmLength - topBaseRadius, 0, topBaseHeight // 2])

bottomBaseHeight = 250
bottomBaseRadius = 140  # diameter: 11 inches measured = 27.94 cm -> Radius ~= 14cm
bottomBasePosition = np.array([defaultArmLength, 0, -bottomBaseHeight])

# TODO: look at the file fetch.urdf 'rosed fetch_descriptor/robot/fetch.urdf'
# TODO: and look at the joints (not links) and complete the measurements from there!
joint_lengths = {
    "bellows_joint": 0,
    "bellows_joint2": 0,
    "elbow_flex_joint": 0.133,  # in meters
    "forearm_roll_joint": 0.197,  # in meters
    "upperarm_roll_joint": 0.219,
    "wrist_flex_joint": 0.1245,
    "wrist_roll_joint": 0.1385,

}


class Fetch:
    # joints is an array in the form [...(jointLength, jointAxis, smallJointLimit, largeJointLimit)...]
    # joints at the front of the array are joints closest to the robot
    # define x to be axis pointing from fetch's face, y to be axis perpendicular to x axis and parallel to ground,
    # and z axis to be pointing up
    # let's just say units are in millimeters
    def __init__(self, initial_position = None):
        if initial_position is None:
            initial_position = np.array([0, 0, 0], dtype=np.float64)
        self.topBasePosition = topBasePosition
        self.topBaseRadius = topBaseRadius
        self.topBaseAxis = np.array([0, 0, -topBaseHeight])

        self.bottomBasePosition = bottomBasePosition
        self.bottomBaseRadius = bottomBaseRadius
        self.bottomBaseAxis = np.array([0, 0, -bottomBaseHeight])

        self.topBaseSegments = (self.topBasePosition, self.topBasePosition + self.topBaseAxis)
        self.bottomBaseSegments = (self.bottomBasePosition, self.bottomBasePosition + self.bottomBaseAxis)

        self.zMin = self.bottomBaseSegments[-1][-1] + armThreshold

        self.segments = [
            # shoulder pan joint
            Segment(defaultArmLength, 'z', r(-92), r(92)),

            # shoulder lift joint
            Segment(defaultArmLength, 'y', r(-87), r(70)),

            # upper arm roll joint: default limits of 0 - 360
            Segment(defaultArmLength, 'x'),

            # elbow flex joint
            Segment(defaultArmLength, 'y', r(-129), r(129)),

            # forearm roll joint: default limits of 0 - 360
            Segment(defaultArmLength, 'x', ),

            # wrist flex joint
            Segment(defaultArmLength, 'y', r(-125), r(125)),

            # wrist roll joint: default limits of 0 - 360
            Segment(defaultArmLength, 'x', ),
        ]

        self.base = initial_position

        for i in range(len(self.segments)):
            if i > 0:
                self.segments[i].parentSegment = self.segments[i - 1]

    def getPoses(self):
        return np.concatenate([[segment.currentRotation for segment in self.segments], self.base])

    def getRandomPoses(self):
        return np.concatenate([[segment.randomRotation() for segment in self.segments],
                               np.random.uniform(-5, 5, (2, )), np.random.uniform(0, 2*np.pi, (1, ))])

    def setRandomPoses(self):
        self.applyPoses(self.getRandomPoses())

    # poseArray is an array of radians
    def applyPoses(self, poseArray):
        for i in range(len(self.segments)):
            self.segments[i].rotate(poseArray[i])
        self.base = np.array(poseArray[-3:])

    def applyRelativePoses(self, relativePoseArray):
        for i in range(len(self.segments)):
            self.segments[i].relativeRotate(relativePoseArray[i])
        self.base += np.array(relativePoseArray[-3:])
        if not (0 <= self.base[2] <= 2*np.pi):
            while self.base[2] < 0:
                self.base[2] += 2*np.pi
            while self.base[2] >= 2*np.pi:
                self.base[2] -= 2*np.pi

    def getSegmentPosition(self, segmentIndex):
        return extractTranslation(self.segments[segmentIndex].computeGlobalTransformationMatrix())

    def getSegmentPositions(self):
        x = np.array([self.getSegmentPosition(i) for i in range(len(self.segments))]).squeeze()
        print(x.shape)
        return x

    def getTool(self, pose=None):
        if pose is not None:
            oldPose = self.getPoses()
            self.applyPoses(pose)
            tool = self.getSegmentPosition(-1)
            self.applyPoses(oldPose)
            return tool

        return self.getSegmentPosition(-1)

    def isPoseValid(self, pose=None):
        if pose is None:
            return self.isCurrentPoseValid()

        oldPose = self.getPoses()
        self.applyPoses(pose)
        valid = self.isCurrentPoseValid()
        self.applyPoses(oldPose)
        return valid

    def isCurrentPoseValid(self):
        segmentPositions = self.getSegmentPositions()

        # I think the joint limits prevent the arms themselves from colliding
        # but it's definitely possible to collide with the base

        # That being said, we'll assume the first joint can't collide with the base due to joint limits

        segmentPairs = []
        for i in range(1, len(self.segments) - 1):
            segmentPairs.append((segmentPositions[i].flatten(), segmentPositions[i + 1].flatten()))

        i = -1
        for segmentPair in segmentPairs:
            # remember, z decreases as we go up
            if segmentPair[0][-1] < self.zMin or segmentPair[1][-1] < self.zMin:
                return False

            pa, pb, topDistance = closestDistanceBetweenLines(*(self.topBaseSegments + segmentPair))
            pa, pb, bottomDistance = closestDistanceBetweenLines(*(self.bottomBaseSegments + segmentPair))

            aboveTopBase = (self.topBaseSegments[0][-1] < segmentPair[0][-1]
                            and self.topBaseSegments[0][-1] < segmentPair[1][-1]
                            and topDistance > armThreshold)

            aboveBottomBase = (self.bottomBaseSegments[0][-1] < segmentPair[0][-1]
                               and self.topBaseSegments[0][-1] < segmentPair[1][-1]
                               and bottomDistance > armThreshold)

            i += 1
            if topDistance - topBaseRadius - armThreshold <= 0 and (not aboveTopBase):
                return False

            if bottomDistance - bottomBaseRadius - armThreshold <= 0 and (not aboveBottomBase):
                return False

        return True

    # destination is a 3x1 xyz
    def getPoseDeltas(self, destination, delta=1e-2):
        initialToolPos = self.getTool()
        currentError = np.vstack([destination - initialToolPos, 1])  # 4x1

        jac = []

        for i in range(len(self.segments)):
            self.segments[i].relativeRotate(delta)
            currentToolPos = self.getTool()
            self.segments[i].relativeRotate(-delta)
            toolDelta = np.vstack([currentToolPos - initialToolPos, 1])  # 4x1
            jac.append(toolDelta)
        # TODO: add the jacobians for the base link (x, y, theta) somehow....

        jac = np.array(jac)[:, :, 0]

        return jac.dot(currentError)

    # goal is a 3x1 np array
    # if verbose is 0, shh
    # if verbose is 1, just print batch stats
    # if verbose is 2, print errors every 5 samples
    def inverseKinematics(self, goal, batchSize=1000, errorThresh=5, verbose=1):
        assert(batchSize > 0)
        currentError = None

        initialPoses = self.getPoses()

        while currentError is None or currentError > errorThresh:
            if verbose > 0:
                print("SGD Batch")
            self.setRandomPoses()

            for i in range(batchSize):
                deltas = self.getPoseDeltas(goal)  # TODO: modify this to return 10D vector (include jacs for the base)
                currentError = np.linalg.norm(goal - self.getTool())

                # scalar represents the amount of the normalized delta vector we want to apply
                scalar = 10 ** (-1 * max(5 - np.log(currentError), 1))
                scaledDeltas = (deltas / np.linalg.norm(deltas)) * scalar
                scaledDeltas = np.array(scaledDeltas.tolist()).flatten()
                self.applyRelativePoses(scaledDeltas)

                if i % 5 == 0 and verbose > 1:
                    print(currentError)

                if currentError < errorThresh:
                    if not self.isPoseValid():
                        if verbose > 0:
                            print("Found invalid pose; trying to find a new pose")
                        continue
                    else:
                        break

            if verbose > 0 and currentError > errorThresh:
                print("This batch resulted in an error of", currentError, "- rerunning from a random pose")

        goalPoses = self.getPoses()
        self.applyPoses(initialPoses)  # TODO: ?!?! huh, why is it not the goal pose?

        if verbose > 0:
            print("Finished with an error of", currentError)

        return goalPoses, currentError
