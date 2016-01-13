#!/usr/bin/env python

import rospy
from code import interact
from geometry_msgs.msg import Twist
from assignment1.srv import *

class directions(object):
    def __init__(self):
        self.twistMessage = Twist()
        """
        self.location_subscriber = rospy.Subscriber("location", 
                                                     Twist, 
                                                     self.callback)
        """
        rospy.init_node("directions_node")
        self.update_location_req = rospy.ServiceProxy("location_srv", location_srv)
        self.request_loc()

    def request_loc(self):
        while True:
            resp_lin_x = float(raw_input("X linear value: " ))
            resp_lin_y = float(raw_input("Y linear value: " ))
            resp_lin_z = float(raw_input("Z linear value: " ))

            resp_ang_x = float(raw_input("X angular value: "))
            resp_ang_y = float(raw_input("Y angular value: "))
            resp_ang_z = float(raw_input("Z angular value: "))
            
            #interact(local=locals())
            resp = self.update_location_req(
                    resp_lin_x,
                    resp_lin_y,
                    resp_lin_z,
                    resp_ang_x,
                    resp_ang_y,
                    resp_ang_z,
            )
            print "response from location node : ",resp.response

if __name__ == '__main__':
    d = directions()
