#!/usr/bin/env python

import serial
import math
import rospy
import struct
from assignment1.srv import *
from assignment1.msg import roomba_odom
from std_msgs.msg import String
from pprint import pprint
import numpy as np

class DriveNode():
    def __init__(self):
        self.connection = None
        self.encoder_max = 65535
        self.drive_struct = struct.Struct('>Bhh')
        self.angle_struct = struct.Struct('>BB')
        self.right_encoder_request, self.left_encoder_request = self.make_angle_request()
        self.port = '/dev/ttyUSB0'
        self.command_dict = {
            'start':self.make_raw_command('128'),
            'safe':self.make_raw_command('131'),
            'passive':self.make_raw_command('128'),
            'full':self.make_raw_command('132'),
            'beep':self.make_raw_command('140 3 1 64 16 141 3'),
            'stop':self.make_raw_command('173'),
            'dock':self.make_raw_command('143'),
            'reset':self.make_raw_command('7')
        }

    def start(self):
        rospy.init_node('drive_node')
        self.command_service = rospy.Service('requestCommand', requestCommand,
                                 self.handle_requestCommand)
        self.drive_service = rospy.Service('requestDrive', requestDrive,
                                 self.handle_requestDrive)
        self.turn_service = rospy.Service('turnAngle', turnAngle,
                                 self.handle_turnAngle)
        self.drive_dist_service = rospy.Service('driveDist', driveDist,
                                 self.handle_driveDist)
        self.angle_service = rospy.Service('requestAngle', requestAngle,
                                           self.handle_requestAngle)
        self.odom_pub = rospy.Publisher(
                'base/odometry',
                roomba_odom,
                queue_size = 10
        )
        self.pose = roomba_odom()
        self.connect_robot()
        self.encoder_count_reset()
        self.odom_loop()

    def odom_loop(self):
        self.odom_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            left_dist, right_dist = self.get_encoder_diffs()
            if (left_dist > self.encoder_max / 10. or
                    right_dist > self.encoder_max / 10):
                print "ENCODER ROLLOVER DETECTED!"
            self.encoder_count_reset()
            dx, dy, da = self.calc_pose_deltas(left_dist, right_dist)
            self.update_pose(dx, dy, da)
            #pprint(self.pose)
            self.odom_pub.publish(self.pose)
            self.odom_rate.sleep()

    def calc_pose_deltas(self, left_dist, right_dist):
        #THIS IS ONLY VALID FOR SMALL TIMESTEPS! (where cos(delta_ang) ~= 1)
        center_dist = (left_dist + right_dist) / 2.
        delta_angle = (right_dist - left_dist) / 2.
        delta_x = center_dist * math.cos(self.pose.angle)
        delta_y = center_dist * math.sin(self.pose.angle)
        return delta_x, delta_y, delta_angle

    def update_pose(self, dx, dy, da):
        self.pose.pos_x += dx
        self.pose.pos_y += dy
        self.pose.angle_tot += da
        self.pose.angle = np.sign(self.pose.angle_tot) * self.pose.angle % 360.

    def get_encoder_diffs(self):
        left_counts, right_counts = self.get_encoder_counts()
        left_diff, right_diff = self.rollover_compensator(left_counts, right_counts)
        left_mm = self.counts_to_mm(left_diff)
        right_mm = self.counts_to_mm(right_diff)
        return left_mm, right_mm

    def rollover_compensator(self, left_counts, right_counts):
        left_diff = left_counts - self.left_total
        right_diff = right_counts - self.right_total
        if abs(left_diff) > self.encoder_max * 0.2:
            #rollover on left encoder
            if left_counts < self.left_total:
                #forward rollover
                left_diff = (self.encoder_max - self.left_total) + left_counts
            if left_counts > self.left_total:
                #backward rollover
                left_diff = (self.left_total - self.encoder_max) - left_counts
        if abs(right_diff) > self.encoder_max * 0.2:
            #rollover on right encoder
            if right_counts < self.right_total:
                #forward rollover
                right_diff = (self.encoder_max - self.right_total) + right_counts
            if right_counts > self.right_total:
                #backward rollover
                right_diff = (self.right_total - self.encoder_max) - right_counts
        return left_diff, right_diff

    def counts_to_mm(self, counts):
        mm = counts * (1/508.8) * (math.pi*72)
        return mm

    def make_drive_command(self, vel, rot):
        #this is to keep vl and vr between -500 and 500 
        vl = sorted([-500, vel + rot, 500])[1]
        vr = sorted([-500, vel - rot, 500])[1]
        cmd = self.drive_struct.pack(145, vr, vl)
        return cmd

    def make_raw_command(self, string):
        cmd = ''
        for num in string.split():
            cmd += chr(int(num))
        print cmd
        return cmd
        
    def connect_robot(self):
        if self.connection is not None:
            print "Already connected!"
            return
        try:
            self.connection = serial.Serial(
                    self.port,
                    baudrate=115200,
                    timeout=1
            )
            print "Connected to robot."
        except:
            print "Connection failed."
        else:
            self.connection.write(self.command_dict['start'])
            self.connection.write(self.command_dict['full'])
            self.connection.write(self.command_dict['beep'])

    def handle_requestCommand(self, request):
        self.connection.write(self.command_dict[request.data])
        return []

    def handle_requestDrive(self, request):
        vel = request.velocity
        rot = request.rotation
        drive_command = self.make_drive_command(vel, rot)
        self.connection.write(drive_command)
        return []

    def make_angle_request(self):
        r_req = self.angle_struct.pack(142, 44)
        l_req = self.angle_struct.pack(142, 43)
        return l_req, r_req

    def encoder_count_reset(self):
        self.connection.write(self.left_encoder_request)
        raw_left_counts = self.connection.read(2)
        left_counts = struct.unpack('>H', raw_left_counts)
        self.connection.write(self.right_encoder_request)
        raw_right_counts = self.connection.read(2)
        right_counts = struct.unpack('>H', raw_right_counts)
        left_counts = left_counts[0]
        right_counts = right_counts[0]
        self.right_total = right_counts
        self.left_total = left_counts

    def handle_driveDist(self, request):
        dist = request.distance
        dist_mm = dist * 25.4   #mm per inch
        dist_counts = dist_mm * (1/(72*math.pi)) * 508.8
        vel = 300
        rot = 0
        drive_command = self.make_drive_command(vel, rot)
        stop_command = self.make_drive_command(0, 0)
        self.encoder_count_reset()
        left_start = self.left_total
        right_start = self.right_total
        self.connection.write(drive_command)
        while (((self.right_total - right_start) < dist_counts) 
               and ((self.left_total - left_start) < dist_counts)):
            rospy.sleep(0.1)
            self.encoder_count_reset()
        self.connection.write(stop_command)
        return "Distance driven."

    def handle_turnAngle(self, request):
        ang_deg = request.degrees
        ang_rad = ang_deg * (math.pi / 180)
        mm_per_wheel = ang_rad * (235 / 2.0)
        rot_per_wheel = mm_per_wheel * (1/(math.pi*72))
        counts_per_wheel = rot_per_wheel * 508.8
        self.encoder_count_reset()
        right_start = self.right_total
        left_start = self.left_total
        drive_command = self.make_drive_command(0, 100)
        stop_command = self.make_drive_command(0, 0)
        self.connection.write(drive_command)
        while (((self.right_total - right_start) < counts_per_wheel)
               and ((left_start - self.left_total) < counts_per_wheel)):
            rospy.sleep(0.1)
            self.encoder_count_reset()
        self.connection.write(stop_command)
        return "Angle turned"

    def get_encoder_counts(self):
        """
        send [142] [Packet ID]
        for angle sensor, packet ID = 20
        will return signed 16 bit value, high byte first
        counterclockwise angles are positive
        value is capped at -32768, +32767
        """
        self.connection.write(self.left_encoder_request)
        raw_left_counts = self.connection.read(2)
        left_counts = struct.unpack('>H', raw_left_counts)
        left_counts = left_counts[0]
        self.connection.write(self.right_encoder_request)
        raw_right_counts = self.connection.read(2)
        right_counts = struct.unpack('>H', raw_right_counts)
        right_counts = right_counts[0]
        return left_counts, right_counts

    def left(self, left_counts, right_counts):
        # this works assuming that:
        #   a) the encoder counts never roll over
        #   b) the wheels always turn by ~ the same ammount
        left_diff = left_counts - self.left_total
        right_diff = right_counts - self.right_total

        left_dist = (left_diff / 508.8) * (math.pi * 72)
        right_dist = right_diff* (1/508.8) * (math.pi*72)

        angle_rad = left_dist / (235/2)
        angle_deg = angle_rad*(180/math.pi)
        return angle_deg

    def right(self, left_counts, right_counts):
        # this works assuming that:
        #   a) the encoder counts never roll over
        #   b) the wheels always turn by ~ the same ammount
        left_diff = left_counts - self.left_total
        right_diff = right_counts - self.right_total

        left_dist = left_diff* (1/508.8) * (math.pi*72)
        right_dist = right_diff* (1/508.8) * (math.pi*72)

        angle_rad = left_dist / (235/2)
        angle_deg = angle_rad*(180/math.pi)
        return angle_deg

    def handle_requestAngle(self, request):
        #print "totals"
        #print (self.left_total, self.right_total)
        left_counts, right_counts = self.get_encoder_counts()
        #print "left, right"
        #print (left_counts, right_counts)
        angle_deg = min(self.left(left_counts, right_counts), self.right(left_counts, right_counts))

        self.right_total = right_counts
        self.left_total = left_counts
        return angle_deg

if __name__ == "__main__":
    driver_node = DriveNode()
    driver_node.start()
