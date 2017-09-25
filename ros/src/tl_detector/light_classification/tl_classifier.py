from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image, boxes):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light with bounding boxes and label
            boxes : bounding coordinates of traffic lights in the image
            top_left x
            top_left y
            bottow right x
            bottom right y 
            there may be mode that one box

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color classification by analyzing the pixels inside the bounding boxes

        for box in boxes:
            if 'traffic light' in box['label']:
                #do something


        return TrafficLight.UNKNOWN
