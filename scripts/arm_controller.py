#!/usr/bin/env python

import serial
import math
import rospy
import struct
from assignment1.srv import armPose

class ArmController():
    def __init__(self):
        self.connection = None
        print self.connection
        #self.arm_struct = struct.Struct('')
        self.port = '/dev/ttyACM0'
        self.ang_len = 2
        rospy.init_node('arm_controller')
        self.pose_service = rospy.Service('armPose', armPose,
                                          self.handle_pose_req)
        self.connect_robot()
        self.arm_state = {
                "M1":0,
                "M2":0,
                "M3":0,
                "M4":0,
                "M5":0
        }
        self.sensor_req = "s"
        self.motor_req = {
                "M1":'m1',
                "M2":'m2',
                "M3":'m3',
                "M4":'m4',
                "M5":'m5'
        }
        #FIX THESE
        self.motor_limits = {
                "M1":(0,0),
                "M2":(0,0),
                "M3":(0,0),
                "M4":(0,0),
                "M5":(0,0)
        }
        #self.update_joint_states()
        rospy.spin()

    def handle_pose_req(self, pose_req):
        self.requested_pose = {
                "M1":pose_req.m1,
                "M2":pose_req.m2,
                "M3":pose_req.m3,
                "M4":pose_req.m4,
                "M5":pose_req.m5
        }
        self.assume_pose()
        return "Pose assumed"

    def assume_pose(self):
        """
        for motor, angle in self.requested_pose:
            self.connection.write(self.motor_req[motor] + angle + '\n')
        """
        self.connection.write('m11010\n')

    def default_pose(self):
        pass

    def vertical_pose(self):
        pass

    def horizontal_pose(self):
        pass

    def update_joint_states(self):
        self.connection.flush()
        self.connection.write('s\n')
        angle_string = self.connection.readline()
        print angle_string
        test = [ord(char) for char in angle_string]
        print test
        angles = angle_string.strip().split()
        angles = [int(angle) for angle in angles]
        for motor, angle in zip(self.arm_state.keys(), angles):
            self.arm_state[motor] = angle

    def connect_robot(self):
        if self.connection is not None:
            print "Already connected!"
            return
        try:
            self.connection = serial.Serial(
                    self.port,
                    timeout=1
            )
            print "Connected to arm."
        except:
            print "Connection failed."
        else:
            self.update_joint_states()
            print "Joint States are: ",self.arm_state
            self.assume_pose()

if __name__=="__main__":
    arm_controller = ArmController()
