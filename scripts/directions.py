#!/usr/bin/env python

import rospy
from code import interact
from geometry_msgs.msg import Twist
from assignment1.srv import *

class directions(object):
    def __init__(self):
        self.twistMessage = Twist()
        #self.location_subscriber = rospy.Subscriber("location", Twist, self.callback)
        rospy.init_node("directions_node")
        rospy.wait_for_service("locationService")

        self.update_location_req = rospy.ServiceProxy("locationService", location_srv)
        self.callback()

    def callback(self):
        while True:
            response_x = float(raw_input("X linear value: " ))
            response_y = float(raw_input("Y linear value: " ))
            response_z = float(raw_input("Z linear value: " ))

            response_x2 = float(raw_input("X angular value: "))
            response_y2 = float(raw_input("Y angular value: "))
            response_z2 = float(raw_input("Z angular value: "))

            self.twistMessage.linear.x = response_x
            self.twistMessage.linear.y = response_y
            self.twistMessage.linear.z = response_z

            self.twistMessage.angular.x = response_x2
            self.twistMessage.angular.y = response_y2
            self.twistMessage.angular.z = response_z2
            resp = self.update_location_req(self.twistMessage)
            print "response from location node : ",resp.response

if __name__ == '__main__':
    d = directions()
