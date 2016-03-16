#!/usr/bin/env python

import os
import rospy
import random
from robot_camera import RobotCamera
from std_msgs.msg import Bool, String
from assignment1.msg import camera_data, roomba_odom, ultrasoundData
from assignment1.srv import *
import math as m
import numpy as np
import copy

class RobotController():
    def __init__(self):
        rospy.init_node("robot_controller")
        self.grab_request = rospy.ServiceProxy("requestGrab", requestGrab)
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
        self.ultrasound_sub = rospy.Subscriber(
                "/Sensors/Ultrasound",
                ultrasoundData,
                self.handle_incoming_ultrasound
        )
        self.arm_mode_pub = rospy.Publisher(
                "/arm_node/mode",
                Bool,
                queue_size = 10
        )

        self.turn_req = rospy.ServiceProxy('turnAngle', turnAngle)
        self.command_req = rospy.ServiceProxy('requestCommand', requestCommand)
        self.drive_request = rospy.ServiceProxy('requestDrive', requestDrive)
        self.arm_cam_on = Bool()
        self.front_cam_on = Bool()
        self.arm_cam_on.data = False
        self.front_cam_on.data = False
        self.seek_speed = 25
        self.collision_threshold = 15.
        self.bucket_dist = 10

        #=================== Environment Variables ============================
        self.mapping_fix = False
        self.odom_estimate = roomba_odom()
        self.odom_home = roomba_odom()
        self.set_home_here()
        self.balls_collected = 0
        self.front_cam = RobotCamera()
        self.arm_cam = RobotCamera()
        self.ultrasound_data = ultrasoundData()
        #this is bad, should use like inheritance or some nonsense
        self.arm_cam.ball_in_grabber = False

    def control_loop(self):
        while not rospy.is_shutdown():
            self.orient_to_home()
            self.competition()
        """
        rospy.sleep(3)
        print "entering control loop"
        self.switch_to_cam("front")
        print "Front cam on"
        print "beeping robot"
        self.beep_robot()
        print "driving to ball"
        self.navigate_randomly_avoid_collisions()
        print "Ball found, beeping robot"
        self.beep_robot()
        print "centering ball"
        self.front_cam_center_ball()
        print "approaching ball"
        self.approach_ball(100)
        print "centering ball again"
        self.front_cam_center_ball()
        print "grabbing ball"
        self.grab_close_ball()
        print "returning home"
        self.goto_waypoint((0,0))
        """

    def competition(self):
        self.navigate_randomly_avoid_collisions()
        self.front_cam_center_ball()
        self.approach_ball()
        self.grab_close_ball()
        bucket = (self.odom_home.pos_x, self.odom_home.pos_y)
        self.goto_bucket_waypoint(bucket)
        self.search_bucket()
        self.approach_bucket()
        self.drop_ball_in_bucket()

    def orient_to_home(self):
        self.search_bucket()
        self.approach_bucket()
        self.turnRight90Degrees()
        self.turnRight90Degrees()
        self.set_home_here()

    def camera_switch(self, camera, value):
        b_value = bool(value)
        if camera == "arm" and self.arm_cam_on.data != b_value:
            self.arm_cam_on.data = b_value
            self.arm_cam_activate_pub.publish(self.arm_cam_on)
        elif camera == "front" and self.front_cam_on.data != b_value:
            self.front_cam_on.data = b_value
            self.front_cam_activate_pub.publish(self.front_cam_on)

    def set_home_here(self):
        self.odom_home = copy.deepcopy(self.odom_estimate)

    #======================= Callback Functions ===============================
    def handle_incoming_ultrasound(self, ultrasound_data):
        self.ultrasound_data = ultrasound_data

    def handle_incoming_arm_cam_data(self, cam_data):
        self.arm_cam = cam_data
        #print "setting arm cam info: ", self.arm_cam.see_ball
        """
        self.arm_cam.see_ball = cam_data.see_ball
        self.arm_cam.ball_pos = cam_data.ball_pos
        self.arm_cam.ball_size = cam_data.ball_size
        """

    def handle_incoming_front_cam_data(self, cam_data):
        self.front_cam = cam_data
        #print "setting front cam info: ", self.front_cam.see_ball
        """
        self.front_cam.see_ball = cam_data.see_ball
        self.front_cam.ball_pos = cam_data.ball_pos
        self.front_cam.ball_size = cam_data.ball_size
        """

    def handle_incoming_odometry(self, odom_message):
        self.odom_estimate = odom_message

    #======================= State Functions ==================================
    def drop_ball_in_bucket(self):
        pass

    def switch_to_cam(self, camera):
        if camera == "front":
            self.camera_switch("arm", 0)
            self.camera_switch("front", 1)
        if camera == "arm":
            self.camera_switch("arm", 1)
            self.camera_switch("front", 0)

    def enable_collision(self):
        bool_msg = Bool()
        bool_msg.data = True
        self.arm_mode_pub.publish(bool_msg)

    def grab_close_ball(self):
        self.camera_switch("arm",1)
        self.camera_switch("front", 0)
        mode = Bool()
        mode.data = True
        self.arm_mode_pub.publish(mode)
        req = False
        self.grab_request(req)

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
        self.turn_to_angle(angle)

    def search_ball(self):
        self.switch_to_cam("front")
        """
        while not self.front_cam.see_ball:
            #look for balls
        """

    def search_bucket(self):
        self.switch_to_cam("front")
        current_angle = self.odom_estimate.angle
        target_angle = current_angle + 360
        self.drive_robot(0, 40)
        while not self.front_cam.see_bucket and self.current_angle < target_angle:
            rospy.sleep(.5)
            current_angle = self.odom_estimate.angle
        self.drive_robot(0,0)
        if self.front_cam.see_bucket:
            return True
        else:
            return False

    def center_bucket(self):
        self.switch_to_cam("front")
        offset = 320 - self.front_cam.bucket_pos[0]
        while offset > 20:
            print "offset", offset
            #turn_rate = max(min(offset, 50), 20)
            turn_rate = self.seek_speed 
            self.drive_robot(0, turn_rate)
            rospy.sleep(0.3)
            self.drive_robot(0, 0)
            rospy.sleep(1.)
            offset = 320 - self.front_cam.bucket_pos[0]
        print "centered bucket, sending stop command"
        self.drive_robot(0, 0)

    def approach_bucket(self):
        self.enable_collision()
        self.switch_to_cam("front")
        self.drive_robot(50, 0)
        while gelf.ultrasound_data.sensor_front > self.bucket_dist:
            rospy.sleep(0.2)
        self.drive_robot(0,0)

    def goto_waypoint(self, waypoint):
        print "pathing to waypoint ", waypoint
        #tolerance is in mm
        self.tolerance = 300.
        waypoint = np.array(waypoint)
        self.orient_toward_waypoint(waypoint)
        print self.odom_estimate
        current_pos = np.array(self.odom_estimate.pos_x, self.odom_estimate.pos_y)
        success = self.drive_to_wp_avoid_col(waypoint, current_pos, False)
        while not success:
            self.moveAroundObjectDetected()
            self.orient_toward_waypoint(waypoint)
            current_pos = np.array(self.odom_estimate.pos_x, self.odom_estimate.pos_y)
            success = self.drive_to_wp_avoid_col(waypoint, current_pos, False)

    def goto_bucket_waypoint(self, waypoint):
        print "pathing to waypoint ", waypoint
        #tolerance is in mm
        self.tolerance = 300.
        waypoint = np.array(waypoint)
        self.orient_toward_waypoint(waypoint)
        print self.odom_estimate
        current_pos = np.array(self.odom_estimate.pos_x, self.odom_estimate.pos_y)
        success = self.drive_to_wp_avoid_col(waypoint, current_pos, True)
        while not success:
            self.moveAroundObjectDetected()
            self.orient_toward_waypoint(waypoint)
            current_pos = np.array(self.odom_estimate.pos_x, self.odom_estimate.pos_y)
            success = self.drive_to_wp_avoid_col(waypoint, current_pos, True)

    def drive_to_wp_avoid_col(self, waypoint, current_pos, bucket):
        self.drive_robot(100, 0)
        while np.linalg.norm(waypoint - current_pos) > self.tolerance and (((not self.front_cam.see_bucket) and bucket) or not bucket):
            if self.ultrasound_data.sensor_front < self.collision_threshold:
                return False
            rospy.sleep(.25)
            current_pos = np.array(self.odom_estimate.pos_x,
                    self.odom_estimate.pos_y)
        self.drive_robot(0, 0)
        return True

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

    #=========================== Navigation ===================================
    def moveAroundObjectDetected(self):
            # Object is detected turn left 90 degrees
            self.rotateLeft90Degress()
            # Now move robot till right ultrasound sensor does not
            # detect any object
            while self.ultrasound_data.sensor_right < (self.collision_threshold + 10):
                    # move the controller a little bit forward at
                    # say 1 second step
                    self.drive_robot(100,0)
                    rospy.sleep(1)
                    self.drive_robot(0,0)

            # So now the right sensor reading is clear, move a bit
            # more forward so that we account for robot`s body length
            # as well
            self.drive_robot(100,0)
            rospy.sleep(2)
            self.drive_robot(0,0)
            # Now we are clear of the object, rotate to the right 90
            # degress so that we are in the same direction
            self.rotateRight90Degrees()

    def rotateLeft90Degress(self):
            current_angle = self.odom_estimate.angle
            target_angle = current_angle + 90
            self.drive_robot(0, 40)
            while(current_angle < target_angle):
                    rospy.sleep(0.25)
                    current_angle = self.odom_estimate.angle
            self.drive_robot(0,0)

    def rotateRight90Degrees(self):
            current_angle = self.odom_estimate.angle
            target_angle = current_angle - 90
            self.drive_robot(0, 40)
            while(current_angle > target_angle):
                    rospy.sleep(0.25)
                    current_angle = self.odom_estimate.angle
            self.drive_robot(0,0)

    def rotateLeftRandom(self, maxTime):
            self.drive_robot(0, 40)
            # Sleep for a random time between 0s to 4s
            rospy.sleep(random.random() * maxTime)
            self.drive_robot(0,0)

    def drive_robotForwardRandom(self, maxTime):
            self.drive_robot(100,0)
            rospy.sleep(random.random() * maxTime)
            self.drive_robot(0,0)

    def turn_to_angle(self, angle):
        current_angle = self.odom_estimate.angle
        self.drive_robot(0, 40)
        while(current_angle > angle):
            rospy.sleep(0.25)
            current_angle = self.odom_estimate.angle
        self.drive_robot(0,0)


    def navigate_randomly(self):
            # Initial state , move forward for a random time first
            # Move randomly till the camera detects a ball
            while not self.front_cam.see_ball:
                    # Sleep for a random time between 0s to 4s
                    self.drive_robotForwardRandom(5);
                    self.rotateLeft90DegreesRandom(2);

    def navigate_randomly_avoid_collisions(self):
        self.enable_collision()
        self.drive_robot(50, 0)
        while not self.front_cam.see_ball:
            rospy.sleep(1)
            if self.ultrasound_data.front_sensor > self.collision_threshold:
                self.rotateLeft90Degrees()
                self.rotateLeftRandom(4)
                self.drive_robot(50, 0)

if __name__ == "__main__":
    rc = RobotController()
    rc.control_loop()
