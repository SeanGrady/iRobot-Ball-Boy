#!/usr/bin/env python
import time
import serial
import math
import rospy
import struct
from cse291_team_2.srv import armPose

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
        self.connect_robot()
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

    def arm_max(self,command):
	
	if(command == 'bot'):
		self.connection.write('m20100')
		self.connection.write('m10100')
		self.connection.write('m30100')
	elif(command == 'top'):
		self.connection.write('m31100')
		self.connection.write('m21100')
		self.connection.write('m11100')
	self.connection.write(command)

    def assume_pose(self):
        """
        for motor, angle in self.requested_pose:
            self.connection.write(self.motor_req[motor] + angle + '\n')
        """
	while(True):
		command = raw_input ("Enter Motor Command: ")
        	if(command == 's'):
			self.connection.write('s')
			print self.connection.readline()
			self.connection.flush()
		else:
			self.arm_max(command)
		#self.connection.write(command)

    def default_pose(self):
        pass

    def vertical_pose(self):
        pass

    def horizontal_pose(self):
        pass

    def update_joint_states(self):

	time.sleep(2)	
	self.connection.write('s')
        print self.connection.readline()
        angle_string = self.connection.readline()
        print angle_string
        angles = angle_string.strip().split()
        angles = [int(angle) for angle in angles]
        print angles
        print self.arm_state.keys()
        for motor, angle in zip(self.arm_state.keys(), angles):
            self.arm_state[motor] = angle

    def connect_robot(self):
        if self.connection is not None:
            print "Already connected!"
            return
        try:
            self.connection = serial.Serial(
                    self.port, 
		    baudrate=9600,
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
