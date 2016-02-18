#!/usr/bin/env python

import rospy

class RobotController():
    def __init__(self):
        rospy.init_node("robot_controller")

    def control_loop(self):
        #need to map out the robot's states and come up with some kind of
        #transition scheme
