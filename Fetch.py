from Segment import Segment
from math import radians as r

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

    # poseArray is an array of radians
    def applyPose(poseArray):
        for i in range(len(self.segments)):
            self.segments[i].rotate(poseArray[i])

    def getTool(self):
        return self.segments[-1].computeGlobalTransformationMatrix()[:-1,-1]

fetch = Fetch()
print(fetch.getTool())