#!/usr/bin/env python
import time
import serial
import math
import rospy
import struct
from std_msgs.msg import Bool, String
from assignment1.srv import *
from assignment1.msg import grabBall, ultrasoundData, camera_data

class ArmController():
    def __init__(self):
        self.arm_cam = camera_data()
        self.connection = None
        self.prev_grab = 0
        #self.arm_struct = struct.Struct('')
        self.port = '/dev/ttyACM0'
        self.ang_len = 2
        self.avoid_collisions = False
        rospy.init_node('arm_controller')
        self.pose_service = rospy.Service('armPose', armPose,
                                          self.handle_pose_req)
        self.arm_cam_sub = rospy.Subscriber(
                "/arm_cam/vision_info",
                camera_data,
                self.handle_incoming_arm_cam_data
        )
        self.grab_service = rospy.Service(
                "requestGrab",
                requestGrab,
                self.handle_grab_toggle
        )
        self.arm_comm_service = rospy.Service(
                "requestArmComm",
                requestArmComm,
                self.handle_arm_comm_req
        )
        self.arm_mode_sub = rospy.Subscriber(
                "/arm_node/mode",
                Bool,
                self.handle_incoming_mode_switch
        )
        self.collision_data_pub = rospy.Publisher(
                "/Sensors/Ultrasound",
                ultrasoundData,
                queue_size = 10
        )
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
    
    def handle_grab_toggle(self, bool_msg):
        print "going to grab ball"
        self.locate_and_grab_ball()
        return []

    def handle_incoming_mode_switch(self, bool_msg):
        print "arm node incoming mode switch", bool_msg
        if bool_msg.data:
            self.avoid_collisions = True
            print "beginning avoidance loop"
            self.collision_avoidance_loop()
        else:
            self.avoid_collisions = False

    def handle_incoming_arm_cam_data(self, cam_data):
        self.arm_cam = cam_data

    def collision_avoidance_loop(self):
        collision_data = self.update_collision_data()
        while self.avoid_collisions:
            collision_data = self.update_collision_data()
            self.collision_data_pub.publish(collision_data)

    def locate_and_grab_ball(self):
        print "centering"
        self.connection.write('c')
        rospy.sleep(2)
        print "going to top"
        self.arm_max('top')
        rospy.sleep(2)
        self.arm_max('drop')
        rospy.sleep(2)
        print "lowering until see ball"
        self.lower_until_ball()
        offset = self.get_ball_x_offset()
        print "offset is ", offset
        while abs(offset) > 30:
            if offset == -380:
                rospy.sleep(.1)
            amt = '003'
            d = int(offset < 0)
            command = 'm4'+str(d)+amt
            print "sending command: ", command
            self.connection.write(command)
            rospy.sleep(1.0)
            offset = self.get_ball_x_offset()
            print "offset is ", offset
        self.pickup_ball()

    def pickup_ball(self):
        self.arm_max('bot')
        rospy.sleep(3)
        self.arm_max('bot')
        rospy.sleep(3)
        self.arm_max('grab')
        rospy.sleep(4)
        self.arm_max('top')
        rospy.sleep(8)

    def get_ball_x_offset(self):
        ball_x = self.arm_cam.ball_pos[0]
        offset = ball_x - (320 + 60)
        return offset
        
    def lower_until_ball(self):
        print "lowering arm"
        self.connection.write('m10010')
        while not self.arm_cam.see_ball:
            rospy.sleep(1.5)
            self.connection.write('m10010')
        self.connection.write('q')
        self.connection.write('m10003')
        print "seen ball"

    def update_collision_data(self):
        collision_data = ultrasoundData()
        if not self.avoid_collisions:
            return collision_data
        left, right, front = self.get_sensor_readings()
        collision_data.sensor_front = front
        collision_data.sensor_left = left
        collision_data.sensor_right = right
        return collision_data

    def control_loop(self):
        while not rospy.is_shutdown():
            self.grab_or_drop()

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
		#self.connection.write('m30100')
		#self.connection.write('m20100')
		self.connection.write('m10100')
	elif(command == 'top'):
		#self.connection.write('m31100')
		#self.connection.write('m21100')
		self.connection.write('m11100')
	elif(command == 'drop'):
		self.connection.write('m30300')
	elif(command == 'grab'):
		self.connection.write('m31300')

    def assume_pose(self):
        """
        for motor, angle in self.requested_pose:
            self.connection.write(self.motor_req[motor] + angle + '\n')
        """
        while not rospy.is_shutdown():
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

    def read_sensors(self):
	self.connection.write('s')
        angle_string = self.connection.readline()
        angles = angle_string.strip().split()
        angles = [int(angle) for angle in angles]
        return angles

    def update_joint_states(self):
	time.sleep(2)	
        angles = self.read_sensors()
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
                    timeout=2
            )
        except:
            print "Connection failed."
        else:
            print "Connected to arm."
            rospy.sleep(2)
            self.arm_max('top')
            rospy.sleep(15)
            self.arm_max('drop')
            rospy.sleep(2)
            #self.control_loop()
            #self.assume_pose()

    # This function gets the sensor readings for all sensors
    def get_sensor_readings(self):
            if not self.avoid_collisions:
                return 0, 0, 100
            # connect to Arduino
            self.connection.write('u1')
            front_raw = self.connection.readline().strip().split()[0]
            if front_raw:
                try:
                    front = int(front_raw)
                except:
                    front = 100
            else:
                front = 100

            """
            self.connection.write('u2')
            left = int(self.connection.readline().strip())

            self.connection.write('u3')
            right = int(self.connection.readline().strip())
            """
            left = 0
            right = 0
            return left, right, front

    # This returns sensor reading only for a given sensor
    def getSensorReading(self, sensor):
            if sensor == 1:
                    self.connection.write('u1')
                    reading = int(self.connection.readline())

            elif sensor == 2:
                    self.connection.write('u2')
                    reading = int(self.connection.readline())

            elif sensor == 3:
                    self.connection.write('u3')
                    reading = int(self.connection.readline())
            else:
                    print "ERROR : wrong sensor id passed"
                    return None
            return reading

    def handle_arm_comm_req(self, req):
        self.arm_max(req.command)
        return []

if __name__=="__main__":
    arm_controller = ArmController()
