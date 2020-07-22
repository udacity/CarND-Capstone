import os
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
from PIL import Image

import util

class traffic_light_detector(object):
    def __init__(self, model="ssd_inception_v2_coco_2017_11_17"):
        util.prepare_tensorflow_object_detection_api(model=model)
        self.predictor_fn = tf.contrib.predictor.from_saved_model(
            export_dir=os.path.join(model, "saved_model"),
            signature_def_key="serving_default")

    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            # the class id of traffic lights is 10
            if scores[i] >= min_score and classes[i] == 10:
                idxs.append(i)
        
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes        

    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].
        
        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width
        return box_coords
        
    def predict(self, image, confidence_cutoff=0.4):
        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)
        result = self.predictor_fn({'inputs': image_np})
        boxes = np.squeeze(result["detection_boxes"])
        scores = np.squeeze(result["detection_scores"])
        classes = np.squeeze(result["detection_classes"])
        boxes, scores, classes = self.filter_boxes(confidence_cutoff,
                                                   boxes, scores, classes)

        width, height = image.size
        box_coords = self.to_image_coords(boxes, height, width)
        return box_coords, classes
    
if __name__ == "__main__":
    tld = traffic_light_detector()
    image = Image.open('test_images/sample1.jpg')
    box_coords, classes = tld.predict(image)
    util.draw_boxes(image, box_coords, thickness=2)

    plt.imshow(np.asarray(image, dtype=np.uint8))
    plt.title(classes)
    plt.show()

