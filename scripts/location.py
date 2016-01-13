#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from assignment1.srv import *

class location(object):
    def __init__(self):
        self.twistMessage = Twist()
        self.location_publisher = rospy.Publisher("location", Twist, queue_size = 10)
        rospy.init_node("location_node")
        self.location_service = rospy.Service('locationService', location_srv, self.handle_location)

    def handle_location(self, incomingT):
        self.twistMessage = incomingT.locationReq
        return "success"

    def publish(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.location_publisher.publish(self.twistMessage)
            rate.sleep()

if __name__ == '__main__':
    l = location()
    l.publish()