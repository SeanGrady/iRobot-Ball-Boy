#!/usr/bin/env python

import rospy
from robot_camera import RobotCamera
from std_msgs.msg import Bool, String
from assignment1.msg import camera_data, roomba_odom
from assignment1.srv import *
import math as m
import numpy as np

class RobotController():
    def __init__(self):
        rospy.init_node("robot_controller")
        self.arm_cam_activate_pub = rospy.Publisher(
                "/arm_cam/activation",
                Bool,
                queue_size = 10
        )
        self.front_cam_activate_pub = rospy.Publisher(
                "/front_cam/activation",
                Bool,
                queue_size = 10
        )
        self.arm_cam_sub = rospy.Subscriber(
                "/arm_cam/vision_info",
                camera_data,
                self.handle_incoming_arm_cam_data
        )
        self.front_cam_sub = rospy.Subscriber(
                "/front_cam/vision_info",
                camera_data,
                self.handle_incoming_front_cam_data
        )
        self.odom_sub = rospy.Subscriber(
                "/base/odometry",
                roomba_odom,
                self.handle_incoming_odometry
        )
        self.turn_req = rospy.ServiceProxy('turnAngle', turnAngle)
        self.command_req = rospy.ServiceProxy('requestCommand', requestCommand)
        self.drive_request = rospy.ServiceProxy('requestDrive', requestDrive)
        self.arm_cam_on = Bool()
        self.front_cam_on = Bool()
        self.arm_cam_on.data = False
        self.front_cam_on.data = False
        self.seek_speed = 20
        #=================== Environment Variables ============================
        self.mapping_fix = False
        self.odom_estimate = roomba_odom()
        self.balls_collected = 0
        self.front_cam = RobotCamera()
        self.arm_cam = RobotCamera()
        #this is bad, should use like inheritance or some nonsense
        self.arm_cam.ball_in_grabber = False

    def control_loop(self):
        print "entering control loop"
        rospy.sleep(3)
        self.camera_switch("arm", 0)
        self.camera_switch("front", 1)
        print "Front cam on"
        print "beeping robot"
        self.beep_robot()
        print "driving to ball"
        self.drive_until_ball()
        print "beeping robot"
        self.beep_robot()
        print "centering ball"
        self.front_cam_center_ball()
        print "approaching ball"
        self.approach_ball(100)
        print "centering ball again"
        self.front_cam_center_ball()
        print "returning home"
        self.goto_waypoint((0,0))

    def camera_switch(self, camera, value):
        b_value = bool(value)
        if camera == "arm" and self.arm_cam_on.data != b_value:
            self.arm_cam_on.data = b_value
            self.arm_cam_activate_pub.publish(self.arm_cam_on)
        elif camera == "front" and self.front_cam_on.data != b_value:
            self.front_cam_on.data = b_value
            self.front_cam_activate_pub.publish(self.front_cam_on)

    #======================= Callback Functions ===============================
    def handle_incoming_arm_cam_data(self, cam_data):
        if self.arm_cam.see_ball != cam_data.see_ball:
            print "setting arm cam info: ", cam_data.see_ball
            self.arm_cam.see_ball = cam_data.see_ball
            self.arm_cam.ball_pos = cam_data.ball_pos
            self.arm_cam.ball_size = cam_data.ball_size

    def handle_incoming_front_cam_data(self, cam_data):
        if self.front_cam.see_ball != cam_data.see_ball:
            print "setting front cam info: ", cam_data.see_ball
            self.front_cam.see_ball = cam_data.see_ball
            self.front_cam.ball_pos = cam_data.ball_pos
            self.front_cam.ball_size = cam_data.ball_size

    def handle_incoming_odometry(self, odom_message):
        self.odom_estimate = odom_message

    #======================= State Functions ==================================
    def drive_until_ball(self):
        self.camera_switch("arm",0)
        self.camera_switch("front", 1)
        while not self.front_cam.see_ball:
            self.drive_robot(100, 0)
            rospy.sleep(.1)
        self.drive_robot(0, 0)

    def beep_robot(self):
        rospy.wait_for_service('requestCommand')
        try:
            self.command_req('beep')
        except ropsy.ServiceException, e:
            print e

    def get_object_in_view(self, obj):
        if obj == "ball":
            self.drive_robot(0, self.seek_speed)
            while not self.front_cam.see_ball:
                rospy.sleep(.01)
        elif obj == "bucket":
            self.drive_robot(0, self.seek_speed)
            while not self.front_cam.see_bucket:
                rospy.sleep(0.01)
        print obj + " is in view"
        self.drive_robot(0, 0)

    def drive_robot(self, velocity, rotation):
        rospy.wait_for_service('requestDrive')
        try:
            self.drive_request(velocity, rotation)
        except rospy.ServiceException, e:
            print e

    def front_cam_center_ball(self):
        self.camera_switch("arm", 0)
        self.camera_switch("front", 1)
        offset = 320 - self.front_cam.ball_pos[0]
        while offset > 20:
            print "offset", offset
            #turn_rate = max(min(offset, 50), 20)
            turn_rate = self.seek_speed 
            self.drive_robot(0, turn_rate)
            rospy.sleep(0.10)
            self.drive_robot(0, 0)
            rospy.sleep(1.)
            offset = 320 - self.front_cam.ball_pos[0]
        print "centered ball, sending stop command"
        self.drive_robot(0, 0)

    def recover_map_fix(self):
        self.camera_switch("arm",0)

    def locate_home(self):
        self.camera_switch("arm",0)
    
    def return_home(self):
        pass

    def orient_toward_waypoint(self, waypoint):
        target_x = waypoint[0]
        target_y = waypoint[1]
        current_x = self.odom_estimate.pos_x
        current_y = self.odom_estimate.pos_y
        current_ang = self.odom_estimate.angle
        slope = (target_y - current_y) / (target_x - current_x)
        angle = m.atan(slope)
        self.turn_req(angle)

    def search_ball(self):
        self.camera_switch("arm",0)
        self.camera_switch("front", 1)
        """
        while not self.front_cam.see_ball:
            #look for balls
        """

    def search_bucket(self):
        self.camera_switch("arm",0)
        """
        while not self.front_cam.see_ball:
            #look for balls
        """

    def goto_waypoint(self, waypoint):
        print "pathing to waypoint ", waypoint
        #tolerance is in mm
        tolerance = 300.
        waypoint = np.array(waypoint)
        self.orient_toward_waypoint(waypoint)
        print self.odom_estimate
        current_pos = np.array(self.odom_estimate.pos_x, self.odom_estimate.pos_y)
        while np.linalg.norm(waypoint - current_pos) > tolerance:
            self.drive_robot(100, 0)
            rospy.sleep(.1)
            current_pos = np.array(self.odom_estimate.pos_x,
                    self.odom_estimate.pos_y)
        print "Arrived at waypoint"

    def approach_ball(self, r_thresh):
        self.camera_switch('front', True)
        self.camera_switch('arm', False)
        while self.front_cam.see_ball:
            if self.front_cam.ball_size < r_thresh:
                self.drive_robot(50, 0)
                rospy.sleep(0.1)
            else:
                self.drive_robot(0,0)
                break

    def pickup_ball(self):
        #this is going to be a hard one...
        self.camera_switch("arm",1)
        """
        while self.arm_cam.see_ball and not self.arm_cam.ball_in_grabber:
        """

    def return_bucket(self):
        self.camera_switch("arm",0)
        """
        while not self.front_cam.see_bucket:
            #look for bucket (using map to go back to home?)
        while self.front_cam.see_bucket:
            #center bucket
            #move toward bucket
        """
        pass

if __name__ == "__main__":
    rc = RobotController()
    rc.control_loop()
