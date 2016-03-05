#!/usr/bin/env python

import rospy
from robot_camera import RobotCamera
from std_msgs.msg import Bool, String
from assignment1.msg import camera_data
from assignment1.srv import *

class RobotController():
    def __init__(self):
        rospy.init_node("robot_controller")
        self.arm_cam_activate_pub = rospy.Publisher(
                "/arm_camera/activation",
                Bool,
                queue_size = 10
        )
        self.arm_cam_sub = rospy.Subscriber(
                "/arm_camera/data",
                camera_data,
                self.handle_incoming_arm_cam_data
        )
        self.front_cam_sub = rospy.Subscriber(
                "/front_camera/data",
                camera_data,
                self.handle_incoming_front_cam_data
        )
        self.command_req = rospy.ServiceProxy('requestCommand', requestCommand)
        self.arm_cam_on = Bool()
        self.arm_cam_on.data = False
        #=================== Environment Variables ============================
        self.mapping_fix = False
        self.balls_collected = 0
        self.forward_cam = RobotCamera()
        self.arm_cam = RobotCamera()
        #this is bad, should use like inheritance or some shit
        self.arm_cam.ball_in_grabber = False

    def control_loop(self):
        while True:
            if self.arm_cam.see_ball:
                rospy.wait_for_service('requestCommand')
                try:
                    self.command_req('Beep')
                except ropsy.ServiceException, e:
                    print e

    def arm_camera_switch(self, value):
        b_value = bool(value)
        if self.arm_cam_on.data != b_value:
            self.arm_cam_on.data == b_value
            self.arm_cam_activate_pub.publish(self.arm_cam_on)

    #======================= Callback Functions ===============================
    def handle_incoming_arm_cam_data(self, cam_data):
        self.arm_cam.see_ball = cam_data.see_ball
        self.arm_cam.ball_centered = cam_data.ball_centered
        self.arm_cam.ball_size = cam_data.ball_size

    def handle_incoming_front_cam_data(self, cam_data):
        #fill self.front_cam object accordingly
        pass

    #======================= State Functions ==================================
    def recover_map_fix(self):
        self.arm_camera_switch(0)

    def locate_home(self):
        self.arm_camera_switch(0)

    def search_ball(self):
        self.arm_camera_switch(0)
        """
        while not self.forward_cam.see_ball:
            #look for balls
        """

    def search_bucket(self):
        self.arm_camera_switch(0)
        """
        while not self.forward_cam.see_ball:
            #look for balls
        """

    def approach_ball(self):
        self.arm_camera_switch(1)
        while self.forward_cam.see_ball and not self.arm_cam.see_ball:
            if self.forward_cam.ball_centered:
                #drive forward
                pass
            else:
                #center ball
                pass

    def pickup_ball(self):
        #this is going to be a hard one...
        self.arm_camera_switch(1)
        """
        while self.arm_cam.see_ball and not self.arm_cam.ball_in_grabber:
        """

    def return_bucket(self):
        self.arm_camera_switch(0)
        """
        while not self.forward_cam.see_bucket:
            #look for bucket (using map to go back to home?)
        while self.forward_cam.see_bucket:
            #center bucket
            #move toward bucket
        """
        pass

if __name__ == "__main__":
    rc = RobotController()
    rc.control_loop()
