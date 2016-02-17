#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
import tf

class PositionTracker():
    def __init__(self):
        rospy.init_node('position_tracker')
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(0.5)
        self.track_position()

    def track_position(self):
        while not rospy.is_shutdown():
            try:
                trans, rot = self.listener.lookupTransform(
                        '/ORB_SLAM/Camera',
                        '/ORB_SLAM/World',
                        rospy.Time(0)
                )
                print trans, rot
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.rate.sleep()
                continue
            self.rate.sleep()

if __name__=="__main__":
    pt = PositionTracker()
