#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import colorsys
from code import interact
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped

ball_hsv_color = (6, 138, 190)
ball_threshold = (5, 10, 10)
openKernSizeForClose = 80
openKernSizeForFar = 40
closeKernSizeForClose = 50
closeKernSizeForFar = 30

class ObjectRecognizer():
    def __init__(self):
        self.bridge = CvBridge()
        self.pointMessage = PointStamped()
        self.latest_disp = None
        self.image_pub = rospy.Publisher("/Object_Recognizer_Node/Boxed",
                                         Image,
                                         queue_size = 10)
        self.rviz_ball_pub = rospy.Publisher("/rviz_points/ball",
                                         PointStamped,
                                         queue_size = 10)
        self.rviz_cam1_pub = rospy.Publisher("/rviz_points/camera_2",
                                         PointStamped,
                                         queue_size = 10)
        self.rviz_cam2_pub = rospy.Publisher("/rviz_points/camera_1",
                                         PointStamped,
                                         queue_size = 10)
        rospy.init_node("object_recognizer")
        self.rectified_sub= rospy.Subscriber(
            "/my_stereo/right/image_rect_color",
            Image,
            self._handle_incoming_rect
        )
        self.disp_sub = rospy.Subscriber(
                "/my_stereo/disparity",
                DisparityImage,
                self._handle_incoming_disp
        )
        rospy.spin()

    def _handle_incoming_disp(self, disp_im):
        image = self.bridge.imgmsg_to_cv2(disp_im.image, desired_encoding="passthrough")
        self.latest_disp = image
        self.latest_disp_params = disp_im

    def _handle_incoming_rect(self, rect_im):
        hsv_image = self._convert_raw_2_hsv(rect_im)
        drawn_image, bx, by = self._find_ball(hsv_image)
        if bx < self.latest_disp.shape[0] and by < self.latest_disp.shape[1]:
            disp = self.latest_disp[bx, by]
            print "Disp at ball: ", disp
            depth = self._find_depth(disp, bx, by)
            print "depth at ball: ", depth
        self.pointMessage.header.stamp = rospy.Time.now()
        self.pointMessage.header.frame_id = "map"
        self.pointMessage.point.x = 0
        self.pointMessage.point.y = 0
        self.pointMessage.point.z = 0
        self.rviz_cam1_pub.publish(self.pointMessage)
        self.pointMessage.header.stamp = rospy.Time.now()
        self.pointMessage.header.frame_id = "map"
        self.pointMessage.point.x = 1
        self.pointMessage.point.y = 0
        self.pointMessage.point.z = 0
        self.rviz_cam2_pub.publish(self.pointMessage)
        rgb_image = cv2.cvtColor(drawn_image, cv2.COLOR_HSV2BGR)
        ros_image = self.bridge.cv2_to_imgmsg(rgb_image, "bgr8")
        self.image_pub.publish(ros_image)

    def _find_depth(self, disp, x, y):
        B = self.latest_disp_params.T
        f = self.latest_disp_params.f
        depth = (B * f)/disp
        return depth

    def _convert_raw_2_hsv(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        return hsv_image

    def _find_ball(self, hsv_image):
        bx, by, bw, bh, bMask = self._find_rect(
                hsv_image,
                ball_hsv_color,
                ball_threshold,
                openKernSizeForClose,
                closeKernSizeForFar
        )
        ball_cent_x = round(bx + (bw/2))
        ball_cent_y = round(by + (bh/2))
        print "ball center is: (", ball_cent_x, "," , ball_cent_y, ")\n"
        bPoint1, bPoint2 = (bx, by), (bx+bw, by+bh)
        cv2.rectangle(hsv_image, bPoint1, bPoint2, [255, 255, 255], 2)
        return hsv_image, ball_cent_x, ball_cent_y

    def _find_rect(self, frame, color, threshold, closeKernSize, openKernSize):
        frame = self._threshold_image(frame, color, threshold)
        kernel = np.ones((closeKernSize,closeKernSize), np.uint8)
        frame = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, kernel)
        kernel = np.ones((openKernSize,openKernSize), np.uint8)
        frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
        contour_struct = cv2.findContours(
                frame,
                cv2.RETR_LIST,
                cv2.CHAIN_APPROX_NONE
        )
        contours = contour_struct[0]
        if len(contours) > 0:
            x, y, w, h = cv2.boundingRect(contours[0])
            return x, y, w, h, frame
        else:
            return -1, -1, -1, -1, -1

    def _threshold_image(self, hsv_image, color, threshold):
        hsv_upper = (color[0]+threshold[0], color[1]+threshold[1], color[2]+threshold[2])
        hsv_lower = (color[0]-threshold[0], color[1]-threshold[1], color[2]-threshold[2])
        binary_image = cv2.inRange(hsv_image, hsv_lower, hsv_upper)
        return binary_image

if __name__ == "__main__":
    object_rec = ObjectRecognizer()
