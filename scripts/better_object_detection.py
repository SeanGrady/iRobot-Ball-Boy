#!/usr/bin/env python

import numpy as np
import sys
import argparse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import rospy
import imutils
from assignment1.msg import camera_data
from collections import deque
import cv2
from circles_buffer import CirclesBuffer, CirclesStruct

class VisionConstants:
    def __init__(self):
        self.camera_active = False
        self.hsv_lower = (26, 75, 46)
        self.hsv_upper = (58, 255, 255)
        self.blur_size = 9
        self.hough_accumulator = 1
        self.hough_min_dist = 100
        self.hough_radius_min = 10
        self.hough_radius_max = 600
        self.hough_param1 = 50
        self.hough_param2 = 40

        openKernSize = 20
        closeKernSize = 5
        self.close_kernel = np.ones((closeKernSize,closeKernSize), np.uint8)
        self.open_kernel = np.ones((openKernSize,openKernSize), np.uint8)

class CamVision():
    def __init__(self):
        #define many things. Not sure if this would be better somewhere else
        rospy.init_node("ball_detector")
        self.parser = argparse.ArgumentParser()
        #camera_type should be either 'arm' or 'front'
        self.parser.add_argument('camera_type')
        self.parser.parse_args(namespace=self)
        self.init_funcs(self.camera_type)
        rospy.spin()

    def init_funcs(self):
        self.init_pubsubs()
        #self.init_vision_constants()
        self.constants = VisionConstants()
        self.init_opencv_things()
        if self.camera_type == "arm":
            self.init_arm_cam_constants()
        elif self.camera_type == "front":
            self.init_front_cam_constants()
        else:
            print "Camera type not recognized..."
            sys.exit()

    def init_opencv_things(self):
        self.circle_struct = CirclesStruct(20)
        self.bridge = CvBridge()
    
    def init_arm_cam_constants(self):
        self.grab_buffer = deque(maxlen=20)
        self.grab_xrange = range(0, 640)
        self.grab_yrange = range(0, 480)
        self.grab_size = 100

    def init_front_cam_constants(self):
        #probably things to do with the bucket
        pass

    def init_pubsubs(self):
        self.raw_image_sub = rospy.Subscriber(
                "/image_raw",
                Image,
                self.handle_incoming_image
        )
        self.camera_activation_sub = rospy.Subscriber(
                "/activation",
                Bool,
                self.handle_activation_message
        )
        self.image_pub = rospy.Publisher(
                "/circled_image",
                Image,
                queue_size = 10
        )
        self.mask_pub = rospy.Publisher(
                "/masked_image",
                Image,
                queue_size = 10
        )
        self.camera_pub = rospy.Publisher(
                "/vision_info",
                camera_data,
                queue_size = 10
        )

    def handle_activation_message(self, message):
        if message.data:
            self.constants.camera_active = True
        else:
            self.constants.camera_active = False

    def handle_incoming_image(self, ros_image):
        if self.constants.camera_active:
            image = self.bridge.imgmsg_to_cv2(
                    ros_image,
                    desired_encoding="bgr8"
            )
            cam_info = self.build_camera_info(image)
            self.camera_pub.publish(cam_info)

    def build_camera_info(self, image):
        #image should be a bgr8 cv2 image
        circles = self.color_circles(image)
        self.circle_struct.add(circles)
        self.update_circle_averages(circles)
        cam_info = camera_data()
        if self.camera_type == "arm":
            see_ball, ball_centered, ball_size = self.get_ball_info()
            cam_info.see_ball = see_ball
            cam_info.ball_centered = ball_centered
            cam_info.ball_size = ball_size
            return cam_info
        elif self.camera_type == "front":
            pass

    def get_ball_info(self):
        if self.circle_struct[0].avg[2] > 5:
            see_ball = True
        return see_ball, False, 0

    def time_avg_circles(self, circles):
        see_ball = 0
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for x, y, r in circles:
                #print x, y, r
                if x in self.grab_xrange and y in self.grab_yrange and r > self.grab_size:
                    see_ball = 1
        self.grab_buffer.append(see_ball)
        ball_ready = 0
        if sum(self.grab_buffer) >= 10:
            ball_ready = 1
        return ball_ready

    def update_circle_averages(self, circles):
        #going to write this for one circle for now, will need to extend later
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for i, circle in enumerate(circles):
                self.circles[i].add_circle(circle)

    def color_circles(self, image):
        masked_image = self.threshold_color(image)
        circles = self.constants.hough_circles(masked_image, image)
        self.publish_cv_image(circled_image)
        return circled_image, circles

    def threshold_color(self, image):
        #threshold image on color and return result to display
        mask = self.create_hsv_mask(image)
        masked_image = self.mask_image(image, mask)
        self.publish_mask(masked_image)
        return masked_image

    def create_hsv_mask(self, rgb_image):
        #blur frame and convert to HSV colorspace
        #may end up resizing frame if need more FPS on odroid
        #frame = imutils.resize(rgb_image, width=600)
        blurred = cv2.GaussianBlur(
                rgb_image,
                (self.constants.blur_size, self.constants.blur_size),
                0
        )
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        #use processed image to create a mask and return it
        mask = cv2.inRange(hsv, self.constants.hsv_lower, self.constants.hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.constants.close_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.constants.open_kernel)
        mask = cv2.dilate(mask, self.constants.open_kernel)
        return mask

    def mask_image(self, image, mask):
        #There's probably a cleaner way to do this but it's fast
        masked_image = cv2.bitwise_and(image, image, mask = mask)
        return masked_image

    def hough_circles(self, image, raw_image):
        #find circles, draw them on the image, and return result to display
        circles = self.find_circles(image)
        return circled_image, circles

    def find_circles(self, image):
        #Assumes unmasked BGR image, accumulator and min_dist will likely
        #require much tuning. Could add min/max radius in px if required.
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(
                gray_image,
                cv2.cv.CV_HOUGH_GRADIENT,
                self.constants.hough_accumulator,
                self.constants.hough_min_dist,
                param1=self.constants.hough_param1,
                param2=self.constants.hough_param2,
                minRadius=self.constants.hough_radius_min,
                maxRadius=self.constants.hough_radius_max
        )
        return circles

    def draw_circles(self, image, circles):
        #draw all the detected circles, and a box at their centers
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for x, y, r in circles:
                cv2.circle(image, (x, y), r, (0, 0, 255), 4)
                cv2.rectangle(
                        image,
                        (x - 5, y - 5),
                        (x + 5, y + 5),
                        (0, 128, 255),
                        -1
                )
        return image

    def publish_mask(self, mask_image):
        ros_image = self.bridge.cv2_to_imgmsg(mask_image, "bgr8")
        self.mask_pub.publish(ros_image)

    def publish_cv_image(self, cv_image):
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_pub.publish(ros_image)

    def test_cv_func(self, function):
        #apply a function that returns a displayable image to a video stream
        #then displays the results until 'q' is pressed
        while(1):
            ret, image = self.camera.read()
            if ret:
                disp_image = function(image)
                cv2.imshow("image", disp_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        self.camera.release()
        cv2.destroyAllWindows

if __name__ == "__main__":
    cv = CamVision()
