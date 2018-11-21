# Mobile Manipulation Path Planning

## Current State
* Basic segment model of the Fetch
    * Can do inverse kinematics, track tool position
    * Run `Driver.py` to see a test

## Requirements
* Run with Python 3.6
    * The only dependency is currently numpy

## Todo
* Update internal fetch model to have real joint lengths
* Get the 'mobile' part working
    * Add some sort of a base to the fetch object
* Add Bi2RRT*
* Render things in 3D?
    * Or potentially save that for ROS