import numpy as np
import imutils
import cv2

class BallDetector():
    def __init__(self):
        self.hsv_lower = (83, 83, 40)
        self.hsv_upper = (98, 216, 143)
        self.blur_size = 11
        openKernSize = 9
        closeKernSize = 9
        self.close_kernel = np.ones((closeKernSize,closeKernSize), np.uint8)
        self.open_kernel = np.ones((openKernSize,openKernSize), np.uint8)
        self.hough_accumulator = 1.5
        self.hough_min_dist = 80
        self.camera = cv2.VideoCapture(0)

    def create_hsv_mask(self, rgb_image):
        #blur frame and convert to HSV colorspace
        #may end up resizing frame if need more FPS on odroid
        #frame = imutils.resize(rgb_image, width=600)
        blurred = cv2.GaussianBlur(rgb_image, (self.blur_size, self.blur_size), 0)
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
        #Assumes unmasked BGR image
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(
                gray_image,
                cv2.cv.CV_HOUGH_GRADIENT,
                self.hough_accumulator,
                self.hough_min_dist
        )
        return circles

    def draw_circles(self, image, circles):
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for x, y, r in circles:
                cv2.circle(image, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(
                        image,
                        (x - 5, y - 5),
                        (x + 5, y + 5),
                        (0, 128, 255),
                        -1
                )
        return image

    def threshold_color(self, image):
        mask = self.create_hsv_mask(image)
        masked_image = self.mask_image(image, mask)
        return masked_image

    def hough_circles(self, image):
        circles = self.find_circles(image)
        circled_image = self.draw_circles(image, circles)
        return circled_image

    def test_cv_func(self, function):
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
    bd.test_cv_func(bd.threshold_color)
