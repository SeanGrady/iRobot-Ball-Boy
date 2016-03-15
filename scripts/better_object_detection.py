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
        self.camera_active = True
        self.ball_hsv_lower = (40, 106, 66)
        self.ball_hsv_upper = (67, 255, 255)
        self.bucket_hsv_lower = (77, 128, 66)
        self.bucket_hsv_upper = (99, 255, 255)
        self.blur_size = 9
        self.hough_accumulator = 1
        self.hough_min_dist = 100
        self.hough_radius_min = 10
        self.hough_radius_max = 600
        self.hough_param1 = 40
        self.hough_param2 = 30

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
        self.parser.add_argument('ros_things', nargs = '*')
        self.args = self.parser.parse_args()
        self.camera_type = self.args.camera_type
        self.init_funcs()
        rospy.spin()

    def init_funcs(self):
        self.init_debug_consts()
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

    def init_debug_consts(self):
        self.show_circles = False
        self.show_avg_circles = False
        self.show_bucket = True

    def init_opencv_things(self):
        self.circle_struct = CirclesStruct(10)
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
                "camera/image_raw",
                Image,
                self.handle_incoming_image
        )
        self.camera_activation_sub = rospy.Subscriber(
                "activation",
                Bool,
                self.handle_activation_message
        )
        self.image_pub = rospy.Publisher(
                "circled_image",
                Image,
                queue_size = 10
        )
        self.bucket_image_pub = rospy.Publisher(
                "bucket_image",
                Image,
                queue_size = 10
        )
        self.grey_pub = rospy.Publisher(
                "greyscale/camera/image",
                Image,
                queue_size = 10
        )
        self.mask_pub = rospy.Publisher(
                "masked_image",
                Image,
                queue_size = 10
        )
        self.camera_pub = rospy.Publisher(
                "vision_info",
                camera_data,
                queue_size = 10
        )

    def handle_activation_message(self, message):
        print "activation message ", self.camera_type, message
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
            if self.camera_type == "front":
                image = cv2.flip(image, 0)
            #grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            #grey_ros_image = self.bridge.cv2_to_imgmsg(grey_image, "mono8")
            #self.grey_pub.publish(grey_ros_image)
            cam_info, avg_circles, circles, bucket_blob = self.build_camera_info(image)
            self.camera_pub.publish(cam_info)
            if self.show_circles == True:
                circled_image = self.draw_circles(image, circles)
                self.publish_cv_image(circled_image, self.image_pub)
            if self.show_avg_circles == True:
                circled_image = self.draw_circles(image, np.array([avg_circles]))
                self.publish_cv_image(circled_image, self.image_pub)
            if self.show_bucket == True:
                bucket_image = self.render_bucket(image, bucket_blob)
                self.publish_cv_image(bucket_image, self.bucket_image_pub)

    def build_camera_info(self, image):
        #image should be a bgr8 cv2 image
        circles = self.color_circles(image)
        see_bucket = False
        bucket_blob = None
        if self.camera_type == "front":
            bucket_blob = self.detect_bucket(image)
            if bucket_blob is not None:
                see_bucket = True
        cam_info = camera_data()
        self.circle_struct.add_frame_circles(circles)
        avg_circles = self.circle_struct.circles_list[0].avg
        #self.update_circle_averages(circles)
        if self.camera_type == "arm":
            see_ball, ball_pos, ball_size = self.get_ball_info()
            cam_info.see_ball = see_ball
            cam_info.ball_pos = ball_pos
            cam_info.ball_size = ball_size
        elif self.camera_type == "front":
            see_ball, ball_pos, ball_size = self.get_ball_info()
            cam_info.see_ball = see_ball
            cam_info.ball_pos = ball_pos
            cam_info.ball_size = ball_size
            cam_info.see_bucket = see_bucket
            if see_bucket:
                cam_info.bucket_size = bucket_blob[3]
                cam_info.bucket_pos = [bucket_blob[4], bucket_blob[5]]
        return cam_info, avg_circles, circles, bucket_blob

    def get_ball_info(self):
        if self.circle_struct.circles_list[0].bin_avg > 0.75:
            see_ball = True
            ball_xyr = self.circle_struct.circles_list[0].avg
            ball_pos = ball_xyr[0:2]
            ball_size = ball_xyr[2]
        else:
            see_ball = False
            ball_pos = np.array([0,0])
            ball_size = 0
        return see_ball, ball_pos, ball_size

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
                self.circle_struct[i].add_circle(circle)

    def color_circles(self, image):
        masked_image, mask = self.threshold_color(image, self.constants.ball_hsv_lower, self.constants.ball_hsv_upper)
        circles = self.hough_circles(masked_image, image)
        #self.publish_cv_image(circled_image)
        return circles

    def detect_bucket(self, image):
        bucket_blobs = self.detect_bucket_blobs(image)
        biggest_blob = None
        if bucket_blobs is not None:
            biggest_blob = max(bucket_blobs, key=lambda x: x[-1])
        return biggest_blob

    def detect_bucket_blobs(self, image):
        masked_image, mask = self.threshold_color(
                image,
                self.constants.bucket_hsv_lower,
                self.constants.bucket_hsv_upper
        )
        blobs = self.find_contours(mask)
        return blobs

    def find_contours(self, masked_image):
        contour_struct = cv2.findContours(
                masked_image,
                cv2.RETR_LIST,
                cv2.CHAIN_APPROX_NONE
        )
        contours = contour_struct[0]
        blobs = []
        if len(contours)>0: 
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour) 
                area = w*h
                x_c, y_c = self.find_center(x, y, w, h)
                blobs.append((x, y, w, h, int(x_c), int(y_c), area))
        else:
            return None
        return blobs

    def find_center(self, x, y, w, h):
        x_c = x + ((x + w) / 2.) 
        y_c = y + ((y + w) / 2.) 
        return x_c, y_c

    def threshold_color(self, image, hsv_lower, hsv_upper):
        #threshold image on color and return result to display
        mask = self.create_hsv_mask(
                image,
                hsv_lower,
                hsv_upper
        )
        masked_image = self.mask_image(image, mask)
        #self.publish_mask(masked_image)
        return masked_image, mask

    def create_hsv_mask(self, rgb_image, hsv_lower, hsv_upper):
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
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
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
        return circles

    def find_circles(self, image):
        #Assumes unmasked BGR image, accumulator and min_dist will likely
        #require much tuning. Could add min/max radius in px if required.
        grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(
                grey_image,
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
            #circles = np.round(circles[0, :]).astype("int")
            circles = np.round(circles).astype("int")
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

    def render_bucket(self, hsv_image, bucket_blob):
        if bucket_blob is None:
            return hsv_image
        x, y, w, h, x_c, y_c = bucket_blob[:6]
        bPoint1, bPoint2 = (x, y), (x+w, y+h)
        cv2.rectangle(hsv_image, bPoint1, bPoint2, [255, 255, 255], 2)
        cv2.rectangle(
                hsv_image,
                (x_c - 5, y_c - 5),
                (x_c + 5, y_c + 5),
                (0, 128, 255),
                -1
        )
        return hsv_image

    def publish_mask(self, mask_image):
        ros_image = self.bridge.cv2_to_imgmsg(mask_image, "bgr8")
        self.mask_pub.publish(ros_image)

    def publish_cv_image(self, cv_image, publisher):
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        publisher.publish(ros_image)

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
