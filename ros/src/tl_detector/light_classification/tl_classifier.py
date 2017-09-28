from styx_msgs.msg import TrafficLight

import numpy as np
import cv2

class TLClassifierCV(object):
    def __init__(self):
        self.red_bounds = ([0,100,100],[10,255,255])
        self.yellow_bounds = ([18,150,66],[45,255,255])
        self.green_bounds = ([50,128,20],[90,255,255])

    def filter_range(self, img, lower, upper):
        filtered = np.copy(img)
        mask = cv2.inRange(filtered, lower, upper )
        return cv2.bitwise_and(filtered,filtered,mask=mask)

    def remove_noise(self, img):
        denoised_image = np.copy(img)
        #return cv2.fastNlMeansDenoisingColored(denoised_image,None,12,12,7,21)
        return cv2.medianBlur(denoised_image, 3)

    def find_circles(self,img):
        return cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1, 20,
              param1=30,
              param2=10,
              minRadius=10,
              maxRadius=40)

    def local_normalize_histogram(self, img):

        # CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=3., tileGridSize=(8,8))

        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)  # convert from BGR to LAB color space
        l, a, b = cv2.split(lab)  # split on 3 different channels

        l2 = clahe.apply(l)  # apply CLAHE to the L-channel

        lab = cv2.merge((l2,a,b))  # merge channels
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)  # convert from LAB to BGR

    def has_color(self, img_denoise, color='Red'):

        if (color == 'Green'):
            # Filter for green
            img_thresh = self.filter_range(img_denoise,np.array(self.green_bounds[0]),np.array(self.green_bounds[1]))
        elif(color == 'Yellow'):
            # Filter for yellow
            img_thresh = self.filter_range(img_denoise,np.array(self.yellow_bounds[0]),np.array(self.yellow_bounds[1]))
        else:
            # Filter for red
            img_thresh = self.filter_range(img_denoise,np.array(self.red_bounds[0]),np.array(self.red_bounds[1]))


        #self.show_image(img_thresh, title ="filtered {}".format(color))

        # Apply Hough circles
        gray = cv2.cvtColor(np.copy(img_thresh),cv2.COLOR_BGR2GRAY)
        #self.show_image(gray, title ='gray filter')

        circles = self.find_circles(gray)
        #self.show_circles(img_denoise,circles)

        return circles is not None

    def show_image(self,img, title='image'):
        cv2.imshow(title,img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def show_circles(self,img,circles):
        if circles is not None:
        	# convert the (x, y) coordinates and radius of the circles to integers
        	circles = np.round(circles[0, :]).astype("int")

        	# loop over the (x, y) coordinates and radius of the circles
        	for (x, y, r) in circles:
        		# draw the circle in the output image, then draw a rectangle
        		# corresponding to the center of the circle
        		cv2.circle(img, (x, y), r, (0, 255, 0), 4)
        		cv2.rectangle(img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

        	# show the output image
        	cv2.imshow("output", img)
        	cv2.waitKey(0)

    def get_classification(self, image):

        #self.show_image(img, title='original')

        #normalize hist
        img_norm = self.local_normalize_histogram(image);
        #self.show_image(img_norm, title='normalized')

        # Convert to hsv_image
        img_hsv = cv2.cvtColor(np.copy(img_norm), cv2.COLOR_BGR2HSV)
        #self.show_image(img_hsv, title ='hsv')

        # Remove potential noise
        img_denoise = self.remove_noise(img_hsv)
        #self.show_image(img_denoise, title ='denoise')

        has_red = self.has_color(img_denoise,color='Red')

        has_green = self.has_color(img_denoise,color='Green')

        has_yellow = self.has_color(img_denoise,color='Yellow')

        if(has_red):
            return TrafficLight.RED
        elif(has_yellow):
            return TrafficLight.YELLOW
        elif(has_green):
            return TrafficLight.Green
        else:
            return TrafficLight.UNKNOWN


        #print("Light: R:{} G:{} Y:{}".format(has_red,has_green,has_yellow))


class TLClassifierSimple(object):
    def __init__(self):
        self.lower = np.array([150, 100, 150])
        self.upper = np.array([180, 255, 255])

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        state = TrafficLight.UNKNOWN

        red_area, red_image = self.get_colored_area(image, self.lower, self.upper)

        # Needs careful tuning for the number of red pixesls
        red_pixels = 40

        if red_area > red_pixels:
            state = TrafficLight.RED

        return state
    def get_traffic_circle(self,image):
        '''
        Use Canny to get the circles for the images
        '''
        return False


    def get_colored_area(self, image, lower, upper):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask_image = cv2.inRange(hsv_image, lower, upper)
        extracted_image = cv2.bitwise_and(image, image, mask=mask_image)
        area = cv2.countNonZero(mask_image)

        return area, extracted_image
