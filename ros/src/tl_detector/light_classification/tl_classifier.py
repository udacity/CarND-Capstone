from styx_msgs.msg import TrafficLight
import os

cwd = os.getcwd()
os.system('export PYTHONPATH=' + cwd + ':$PYTHONPATH')
from net.build import TFNet

class TLClassifier(object):
    def __init__(self):
		self.options = {"model": cwd + "/cfg/yolov2_ft.cfg",
							 "backup": cwd + "/ckpt/",
							 "labels": 'labels.txt', "load": 3000, "gpu": 1.0}
		self.tfnet = TFNet(self.options)
		self.lightLabel = {'red':TrafficLight.RED, 'green':TrafficLight.GREEN, 'yellow':TrafficLight.YELLOW}

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        _, boxInfo = self.tfnet.return_predict(image)
        if not boxInfo:
        	return TrafficLight.GREEN
        else:
        	label = set()
        	for i in boxInfo:
        		label.add(i['label'])
        	if len(label) == 1:
        		print label
        		return self.lightLabel[label.pop()]

        return TrafficLight.UNKNOWN
