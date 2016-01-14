#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from assignment1.srv import *

class location(object):
    def __init__(self):
        self.twistMessage = Twist()
	self.pointMessage = PointStamped()
        self.location_publisher = rospy.Publisher("location", Twist, queue_size = 10)
	self.point_pub = rospy.Publisher("rviz_point", PointStamped, queue_size= 10)
        rospy.init_node("location_node")
        self.location_service = rospy.Service('locationService', location_srv, self.handle_location)

    def handle_location(self, incomingT):
        self.twistMessage = incomingT.locationReq
	self.pointMessage.header.stamp = rospy.Time.now()
	self.pointMessage.header.frame_id = "map"
	self.pointMessage.point.x = self.twistMessage.linear.x
	self.pointMessage.point.y = self.twistMessage.linear.y
	self.pointMessage.point.z = self.twistMessage.linear.z
        return "success"

    def publish(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.location_publisher.publish(self.twistMessage)
	    self.point_pub.publish(self.pointMessage)
            rate.sleep()

if __name__ == '__main__':
    l = location()
    l.publish()
