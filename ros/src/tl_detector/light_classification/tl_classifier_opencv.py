import rospy
from sensor_msgs.msg import Image
from styx_msgs.msg import TrafficLight
import cv2
import math
import numpy as np


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # pass
        # self.test_image = rospy.Publisher('/test_image', Image, queue_size=1)

        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        #         self.camera_image.encoding = "rgb8"
        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        light = TrafficLight.UNKNOWN

        output = image.copy()
        # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
 
        # define range of luminosity in HSV
        lower_lum = np.array([0,100,200])
        upper_lum = np.array([180,255,255])

        # Threshold the HSV image to get only higher luminosity
        lum_mask = cv2.inRange(hsv, lower_lum, upper_lum)

        # Bitwise-AND mask and original image
        output = cv2.bitwise_and(output,output, mask= lum_mask)

        # define range of blue in HSV
        lower_blue = np.array([80,0, 0])
        upper_blue = np.array([160,255,255])

        # Threshold the HSV image to get only higher luminosity
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        not_blue_mask = cv2.bitwise_not(blue_mask)

        # Bitwise-AND mask and original image
        output = cv2.bitwise_and(output,output, mask= not_blue_mask)

        
    
        hue = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)[:,:,0]

        count_lreds = np.count_nonzero(cv2.inRange(hue,1,19))
        count_ureds = np.count_nonzero(cv2.inRange(hue,160,180))
        count_reds = count_lreds + count_ureds
        count_yellows = np.count_nonzero(cv2.inRange(hue,20,35))
        count_greens = np.count_nonzero(cv2.inRange(hue,40,80))

        count_vec = [count_reds, count_yellows, count_greens]
        max_count_ix = np.argmax(count_vec)
        max_val = count_vec[max_count_ix]

        lights_detected = 'None'

        if (max_val > 120):
            if (max_count_ix == 0):
                lights_detected = 'Red'
                light= TrafficLight.RED
                
            elif (max_count_ix == 1):
                lights_detected = 'Yellow'
                light= TrafficLight.YELLOW
                
            elif (max_count_ix == 2):
                lights_detected = 'Green'
                light= TrafficLight.GREEN
                
        else:
            lights_detected = 'None'
            light= TrafficLight.UNKNOWN           


        # http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
        # img = self.bridge.cv2_to_imgmsg(output, "bgr8")
        
        # for debugging purposes - use 'rqt_image_view' in a separate terminal to see the image
        # self.test_image.publish(img)
        # rospy.loginfo("""Lights detected {}""".format(lights_detected))



        return light
