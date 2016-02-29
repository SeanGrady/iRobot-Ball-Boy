#!/usr/bin/env python

import rospy
from robot_camera import RobotCamera

class RobotController():
    def __init__(self):
        rospy.init_node("robot_controller")
#=========================== Environment Variables ============================
        self.mapping_fix = False
        self.balls_collected = 0
        self.forward_cam = RobotCamera()
        self.arm_cam = RobotCamera()
        #this is bad, should use like inheritance or some shit
        self.arm_cam.ball_in_grabber = False

    def control_loop(self):
        #need to map out the robot's states and come up with some kind of
        #transition scheme

#=========================== State Functions ==================================
    def approach_ball(self):
        while self.forward_cam.see_ball and not self.arm_cam.see_ball:
            if self.forward_cam.ball_centered:
                #drive forward
            else:
                #center ball

    def pickup_ball(self):
        #this is going to be a hard one...
        """
        while self.arm_cam.see_ball and not self.arm_cam.ball_in_grabber:
        """
        pass

    def search(self):
        """
        while not self.forward_cam.see_ball:
            #look for balls
        """
        pass

    def return_bucket(self):
        """
        while not self.forward_cam.see_bucket:
            #look for bucket (using map to go back to home?)
        while self.forward_cam.see_bucket:
            #center bucket
            #move toward bucket
        """
        pass
