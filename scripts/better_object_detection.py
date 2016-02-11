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
        self.camera = cv2.VideoCapture(0)

    def create_hsv_mask(self, rgb_image):
        #resize frame, blur it, and convert to HSV colorspace
        #frame = imutils.resize(rgb_image, width=600)
        blurred = cv2.GaussianBlur(rgb_image, (self.blur_size, self.blur_size), 0)
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        #use processed image to create a mask and return it
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.close_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.open_kernel)
        return mask

    def mask_image(self, image, mask):
        masked_image = cv2.bitwise_and(image, image, mask = mask)
        return masked_image

    def test_mask(self):
        while(1):
            ret, image = self.camera.read()
            if ret:
                mask = self.create_hsv_mask(image)
                masked_image = self.mask_image(image, mask)
                cv2.imshow("image", masked_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        self.camera.release()
        cv2.destroyAllWindows

if __name__ == "__main__":
    bd = BallDetector()
    bd.test_mask()
