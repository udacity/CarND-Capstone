from styx_msgs.msg import TrafficLight

import tl_cnn
import numpy as np

class TLClassifierCT(object):
    def __init__(self):
        #TODO load classifier
        self.skip_factor = 0
        self.out_counter = -1
        self.prev_result = TrafficLight.UNKNOWN

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        self.out_counter += 1
        if self.skip_factor > 0 and self.out_counter % self.skip_factor != 0:
            return self.prev_result

        loc = tl_cnn.run(image)
        threshold = 0.9999
        r = np.sum(loc[0,0,:,:,1]>threshold)
        g = np.sum(loc[0,0,:,:,2]>threshold)
        y = np.sum(loc[0,0,:,:,3]>threshold)

        result = TrafficLight.UNKNOWN
        if(r+g+y)>20:
            if(r>=g and r>=y):
                result = TrafficLight.RED
            elif(g>y and g>15):
                result = TrafficLight.GREEN
            elif(y>10):
                result = TrafficLight.YELLOW
        else:
                result = TrafficLight.UNKNOWN
        
        self.prev_result = result
        return result
