import os
from styx_msgs.msg import TrafficLight
import numpy as np
import rospy
import torch
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
import cv2


class TLClassifier(object):
    def __init__(self, pymodel_file):
        #TODO load classifier
        self.current_light = TrafficLight.UNKNOWN
        cwd = os.path.dirname(os.path.realpath(__file__))

        model_path = os.path.join(cwd, "train_model/{}".format(pymodel_file))
        rospy.logwarn("model_path={}".format(model_path))
        # Load the PyTorch model
        self.model = torchvision.models.detection.fasterrcnn_resnet50_fpn(
            pretrained=True)
        num_classes = 4
        # get number of input features for the classifier
        in_features = model.roi_heads.box_predictor.cls_score.in_features
        # replace the pre-trained head with a new one
        self.model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)

        # Load model from saved pt file
        self.model.load_state_dict(
            torch.load(model_path+'/tlight_90.pt', map_location=torch.device('cpu')))
        self.tl_catogeries = {
                1: TrafficLight.UNKNOWN,
                2: TrafficLight.GREEN,
                3: TrafficLight.YELLOW,
                4: TrafficLight.RED
            }

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #   TODO implement light color prediction
        # Need to implement traffic light color prediction
        # current_light = np.random.choice(
        #     [TrafficLight.UNKNOWN, TrafficLight.RED,
        #     TrafficLight.GREEN, TrafficLight.YELLOW]
        #     )
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        (im_width, im_height, _) = image_rgb.shape
        image_np = np.expand_dims(image_rgb, axis=0)
        rospy.logwarn(image_np.shape)
        outputs = self.model(image_np)
        boxes = np.squeeze(outputs['boxes'])
        scores = np.squeeze(outputs['scores'])
        classes = np.squeeze(outputs['labels']).astype(np.int32)

        min_treshold = 0.6
        filtclasses = [
            classes[i] for i in range(
                boxes.shape[0]) if scores[i] > min_treshold]
        filtclasses, counts = np.unique(filtclasses, return_counts=True)
        current_light = self.tl_catogeries[filtclasses[np.argmax(counts)[0]]]
        return current_light
