import os
from styx_msgs.msg import TrafficLight
import numpy as np
import rospy
import torch
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
import cv2


class TLClassifier(object):
    def __init__(self, pymodel_file='tlight.pt'):
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
        in_features = self.model.roi_heads.box_predictor.cls_score.in_features
        # replace the pre-trained head with a new one
        self.model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
        # Load model from saved pt file
        self.model.load_state_dict(
            torch.load(model_path, map_location=torch.device('cpu')))
        self.model.eval()
        self.tl_catogeries = {
                0: TrafficLight.UNKNOWN,
                1: TrafficLight.GREEN,
                2: TrafficLight.YELLOW,
                3: TrafficLight.RED
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
        current_light = TrafficLight.UNKNOWN
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        (im_width, im_height, _) = image_rgb.shape
        image_np = np.expand_dims(image_rgb, axis=0)/255.0
        rospy.logwarn(image_np.shape)
        image_np = [torch.tensor(image_np, dtype=torch.float).reshape(
                3, image_np.shape[1], image_np.shape[2])]

        # rospy.logwarn("image shape={}".format(image_np[0].shape))
        # rospy.logwarn("image val={}".format(image_np[0]))
        outputs = self.model(image_np)
        boxes = outputs[0]['boxes'].detach().numpy()
        scores = outputs[0]['scores'].detach().numpy()
        classes = outputs[0]['labels'].detach().numpy().astype(np.int32)
        rospy.logwarn("image val={},{},{}".format(boxes, scores, classes))
        min_treshold = 0.6
        if boxes.shape[0] >=1:
            try:
                filtclasses = [
                    classes[i] for i in range(
                        boxes.shape[0]) if scores[i] > min_treshold]
                filtclasses, counts = np.unique(filtclasses, return_counts=True)
                current_light = self.tl_catogeries[filtclasses[np.argmax(counts)]]
            except (KeyError, ValueError):
                current_light = TrafficLight.UNKNOWN
        return current_light
