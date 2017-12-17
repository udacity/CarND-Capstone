import sys
import tensorflow as tf
import numpy as np
from detection import LightDetection
from detection import ImageWrap
from detection import freeze_session
import net
import cv2
import aug
from PIL import Image
from keras.models import load_model
from keras import backend as K1
from keras.layers.core import K as K2
import time

print("Python: " + sys.version)
print("TensorFlow:" + tf.__version__)

K1.set_learning_phase(0)
K2.set_learning_phase(0)

class LightDetectionAndClassification:
    def __init__(self):
        self.det = LightDetection("frozen_inference_graph.pb")
        self.det.load_graph()
        self.classifier_net = net.LightNet(None, False)
        #self.classifier_model = self.classifier_net.create_model()
        self.classifier_model = load_model('model-prod-ck.h5')

        freeze = True

        if (freeze):
            print("Freezing the graph...")
            frozen_graph = freeze_session(K1.get_session(), output_names=[self.classifier_model.output.op.name])

            print("Saving the graph in TF...")
            from tensorflow.python.framework import graph_io
            graph_io.write_graph(frozen_graph, ".", "model_main.pb", as_text=False)
            print("Done...")

        print("Model:")
        print(self.classifier_model)

    def find_biggest(self, boxes):
        max_size = 0
        idx = -1

        for i in range(len(boxes)):
            #bot, left, top, right
            y1, x1, y2, x2 = boxes[i]
            #x1, y1, x2, y2 = boxes[i]
            size = (x2-x1) * (y2-y1)

            if (size>max_size):
                max_size = size
                idx = i

        return boxes[idx] if idx>=0 else None

    def annotate_image(self, img, box, label):
        x1, y1, x2, y2 = box
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, label, (x1, y2+30), font, 1, (0, 255, 0))
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 1)


    def current_milli_time(self):
        return int(round(time.time() * 1000))

    def infer(self, image_wrap, annotate):
        start = self.current_milli_time()
        boxes = self.det.infer(image_wrap)
        t1 = self.current_milli_time()

        if len(boxes) == 0:
            return None, None

        biggest = self.find_biggest(boxes)
        y1,x1,y2,x2 = biggest
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)

        box = (x1, y1, x2, y2)

        img = image_wrap.get_image_bgr()
        traffic_light = img [y1:y2, x1:x2]
        traffic_light = cv2.cvtColor(traffic_light, cv2.COLOR_BGR2GRAY);
        traffic_light_64_64 = cv2.resize(traffic_light, (64, 64), interpolation=cv2.INTER_CUBIC)
        traffic_light_64_64 = aug.resize_to_1_if_required(traffic_light_64_64)

        t2 = self.current_milli_time()
        predictions = self.classifier_model.predict(np.array([traffic_light_64_64]))[0]
        predicted_class = np.argmax(predictions)
        predicted_label = self.classifier_net.data_labes[predicted_class]

        if annotate:
            annotated_image = self.annotate_image(img, box, predicted_label)

        t3 = self.current_milli_time()

        print("Timing: ", (t1-start), (t2-t1), (t3-t2))

        return box, predictions, predicted_class, predicted_label , annotated_image if annotate else img, traffic_light



    def infer_and_save(self, image_file):
        image_wrap = ImageWrap(cv2.imread("assets\\" + image_file), True)

        biggest_box, predictions, prediction_class, prediction_label, annotated, img_box = self.infer(image_wrap, True)

        print("Predictions:" + str(predictions))
        print("Biggest box: " + str(biggest_box) + " - " + str(prediction_class) + " - " + str(prediction_label))

        image_wrap.save("out_infer\\" + image_file)
        cv2.imwrite("out_infer\\roi_" + image_file, img_box)



