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
* Build collision system
    * Integrate code in FetchUtil to check if line segments collect
    * Build line segments by getting positions of segment starts and ends
    * Ensure non-adjacent segments aren't within a certain amount of units from each other
        * The joint limits should account for adjacent segment collisions
* Add Bi2RRT*
* Render things in 3D?
    * Or potentially save that for ROS