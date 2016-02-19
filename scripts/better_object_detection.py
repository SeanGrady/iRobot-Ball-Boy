#!/usr/bin/env python

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
import imutils
from assignment1.msg import grabBall
from collections import deque
import cv2

class BallDetector():
    def __init__(self):
        #define many things. Not sure if this would be better somewhere else
        rospy.init_node("ball_detector")
        self.grab_buffer = deque(maxlen=20)
        self.grab_xrange = range(0, 640)
        self.grab_yrange = range(0, 480)
        self.grab_size = 100
        self.hsv_lower = (26, 75, 46)
        self.hsv_upper = (58, 255, 255)
        self.blur_size = 9
        openKernSize = 20
        closeKernSize = 5
        self.close_kernel = np.ones((closeKernSize,closeKernSize), np.uint8)
        self.open_kernel = np.ones((openKernSize,openKernSize), np.uint8)
        self.hough_accumulator = 1
        self.hough_min_dist = 100
        self.hough_radius_min = 10
        self.hough_radius_max = 600
        self.hough_param1 = 50
        self.hough_param2 = 40
        #self.camera = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.raw_image_sub = rospy.Subscriber(
                "/camera/image_raw",
                Image,
                self.handle_incoming_image
        )
        self.image_pub = rospy.Publisher(
                "/Ball_Detector/circled_image",
                Image,
                queue_size = 10
        )
        self.mask_pub = rospy.Publisher(
                "/Ball_Detector/masked_image",
                Image,
                queue_size = 10
        )
        self.ball_pub = rospy.Publisher(
                "/Ball_Detector/ball_positioned",
                grabBall,
                queue_size = 10
        )
        rospy.spin()

    def create_hsv_mask(self, rgb_image):
        #blur frame and convert to HSV colorspace
        #may end up resizing frame if need more FPS on odroid
        #frame = imutils.resize(rgb_image, width=600)
        blurred = cv2.GaussianBlur(
                rgb_image,
                (self.blur_size, self.blur_size),
                0
        )
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        #use processed image to create a mask and return it
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.close_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.open_kernel)
        mask = cv2.dilate(mask, self.open_kernel)
        return mask

    def mask_image(self, image, mask):
        #There's probably a cleaner way to do this but it's fast
        masked_image = cv2.bitwise_and(image, image, mask = mask)
        return masked_image

    def find_circles(self, image):
        #Assumes unmasked BGR image, accumulator and min_dist will likely
        #require much tuning. Could add min/max radius in px if required.
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(
                gray_image,
                cv2.cv.CV_HOUGH_GRADIENT,
                self.hough_accumulator,
                self.hough_min_dist,
                param1=self.hough_param1,
                param2=self.hough_param2,
                minRadius=self.hough_radius_min,
                maxRadius=self.hough_radius_max
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

    def threshold_color(self, image):
        #threshold image on color and return result to display
        mask = self.create_hsv_mask(image)
        masked_image = self.mask_image(image, mask)
        self.publish_mask(masked_image)
        return masked_image

    def hough_circles(self, image, raw_image):
        #find circles, draw them on the image, and return result to display
        circles = self.find_circles(image)
        circled_image = self.draw_circles(raw_image, circles)
        return circled_image, circles

    def publish_mask(self, mask_image):
        ros_image = self.bridge.cv2_to_imgmsg(mask_image, "bgr8")
        self.mask_pub.publish(ros_image)

    def color_circles(self, image):
        masked_image = self.threshold_color(image)
        circled_image, circles = self.hough_circles(masked_image, image)
        self.publish_cv_image(circled_image)
        return circled_image, circles

    def handle_incoming_image(self, ros_image):
        image = self.bridge.imgmsg_to_cv2(
                ros_image,
                desired_encoding="bgr8"
        )
        ball_ready = self.ball_positioned(image)
        #print ball_ready

    def publish_cv_image(self, cv_image):
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_pub.publish(ros_image)

    def ball_positioned(self, image):
        circled_image, circles = self.color_circles(image)
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
        grab_ball = grabBall()
        grab_ball.in_position = ball_ready
        self.ball_pub.publish(grab_ball)
        #print see_ball, ball_ready
        return ball_ready

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
    bd = BallDetector()
    #bd.test_cv_func(bd.color_circles)
