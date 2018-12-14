#!/usr/bin/env python

from vpython import *
from Fetch import Fetch
import numpy as np
import time

armCurve = None
jointSpheres = None
topCylinder = None
baseCylinder = None
scene.forward = vector(0, 1, 0)
scene.center = vector(500, 0, 0)
scene.ambient = vector(0.5, 0.5, 0.8)

# this is so the z axis increases upwards
scene.camera.up = vector(0, -1, 0)

scene.width = 1000
scene.height = 600

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

    topCylinder = cylinder(pos=fetch.topBasePosition, axis=fetch.topBaseAxis, radius=fetch.topBaseRadius)
    baseCylinder = cylinder(pos=fetch.bottomBasePosition, axis=fetch.bottomBaseAxis, radius=fetch.bottomBaseRadius)


    floorZ = fetch.bottomBaseSegments[1][-1]
    size = 1000
    box(pos=vector(size / 2, 0, floorZ), size=vector(size * 3, size, 0.2), color=vector(1, 0, 0))

def rerender(fetch):
    global armCurve
    global jointSpheres
    points = fetch.getSegmentPositions()
    for i in range(len(points)):
        point = points[i]
        armCurve.modify(i, pos=pointToVector(point))
        jointSpheres[i].pos = pointToVector(point)

