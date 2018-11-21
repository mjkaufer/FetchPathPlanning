from vpython import *
from Fetch import Fetch
import numpy as np
import time

armCurve = None
jointSpheres = None

def pointToVector(point):
    return vector(*point.T.tolist()[0])

def init(fetch):
    global armCurve
    global jointSpheres
    points = fetch.getSegmentPositions()
    curvePoints = []

    for point in points:
        curvePoints.append(pointToVector(point))

    armCurve = curve(curvePoints)
    jointSpheres = [sphere(pos=point, radius=10) for point in curvePoints]
    print(dir(armCurve))

def rerender(fetch):
    global armCurve
    global jointSpheres
    points = fetch.getSegmentPositions()
    for i in range(len(points)):
        point = points[i]
        armCurve.modify(i, pos=pointToVector(point))
        jointSpheres[i].pos = pointToVector(point)