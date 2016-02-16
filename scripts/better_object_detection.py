import numpy as np
import imutils
import cv2

class BallDetector():
    def __init__(self):
        #define many things. Not sure if this would be better somewhere else
        self.hsv_lower = (42, 113, 55)
        self.hsv_upper = (68, 255, 255)
        self.blur_size = 9
        openKernSize = 20
        closeKernSize = 5
        self.close_kernel = np.ones((closeKernSize,closeKernSize), np.uint8)
        self.open_kernel = np.ones((openKernSize,openKernSize), np.uint8)
        self.hough_accumulator = 3
        self.hough_min_dist = 50
        self.hough_radius_min = 10
        self.hough_radius_max = 300
        self.camera = cv2.VideoCapture(0)

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
                self.hough_min_dist
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
        return masked_image

    def hough_circles(self, image, raw_image):
        #find circles, draw them on the image, and return result to display
        circles = self.find_circles(image)
        circled_image = self.draw_circles(raw_image, circles)
        return circled_image

    def color_circles(self, image):
        masked_image = self.threshold_color(image)
        circled_image = self.hough_circles(masked_image, image)
        return circled_image

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
    bd.test_cv_func(bd.color_circles)
