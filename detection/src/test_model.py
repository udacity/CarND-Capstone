import numpy as np
import os
import sys
import tensorflow as tf
import time
from glob import glob
import cv2

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

flags = tf.app.flags
flags.DEFINE_string('model', 'export.pb/frozen_inference_graph.pb', 'Frozen model file')
flags.DEFINE_string('labels', 'models/labelmap.pbtxt', 'Labels file')
flags.DEFINE_string('image_path', 'test/sim', 'Test images path')
flags.DEFINE_string('output_path', 'out/test/sim', 'Test images output path')
flags.DEFINE_integer('classes', 14, 'Number of classes')
FLAGS = flags.FLAGS


def load_labels(label_file, num_classes):
    label_map = label_map_util.load_labelmap(label_file)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=num_classes,
                                                                use_display_name=True)
    category_index = label_map_util.create_category_index(categories)
    return category_index


def load_image_into_numpy_array(image):
    (im_width, im_height) = image.size
    return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)


def load_graph(model_file):
    detection_graph = tf.Graph()

    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()

        with tf.gfile.GFile(model_file, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

    return detection_graph


def process_images(detection_graph, test_images, category_index, output_path):
    print("Starting detection")
    with detection_graph.as_default():
        with tf.Session(graph=detection_graph) as sess:
            # Definite input and output Tensors for detection_graph
            image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

            # Each box represents a part of the image where a particular object was detected.
            detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
            detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = detection_graph.get_tensor_by_name('num_detections:0')
            for image_path in test_images:
                image = Image.open(image_path)
                # the array based representation of the image will be used later in order to prepare the
                # result image with boxes and labels on it.
                image_np = load_image_into_numpy_array(image)
                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image_np, axis=0)

                time0 = time.time()

                # Actual detection.
                (boxes, scores, classes, num) = sess.run(
                    [detection_boxes, detection_scores, detection_classes, num_detections],
                    feed_dict={image_tensor: image_np_expanded})

                time1 = time.time()

                print ("Detection in {}ms on {}".format((time1 - time0) * 1000, image_path))

                boxes = np.squeeze(boxes)
                scores = np.squeeze(scores)
                classes = np.squeeze(classes).astype(np.int32)

                # print("Boxes: {}, scores: {}, classes: {}".format(boxes, scores, classes))

                output_image_file = os.path.join(output_path, image_path.split("/")[-1])
                output_image = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
                height, width, _ = output_image.shape

                min_score_thresh = .50
                for i in range(boxes.shape[0]):
                    if scores is None or scores[i] > min_score_thresh:
                        class_name = category_index[classes[i]]['name']
                        print('{}'.format(class_name), scores[i])

                        pt2 = (int(boxes[i][1] * width), int(boxes[i][0] * height))
                        pt1 = (int(boxes[i][3] * width), int(boxes[i][2] * height))

                        output_image = cv2.rectangle(output_image, pt1, pt2, (0,255,0), 5)

                print("Writing to {}".format(output_image_file))
                cv2.imwrite(output_image_file, output_image)


def main(args):
    if not os.path.isdir(FLAGS.output_path):
        print("Creating output dir {}".format(FLAGS.output_path))
        os.makedirs(FLAGS.output_path)

    category_index = load_labels(FLAGS.labels, FLAGS.classes)

    detection_graph = load_graph(FLAGS.model)

    test_images = glob(os.path.join(FLAGS.image_path, '*.jpg'))
    print("Length of test images:", len(test_images))

    process_images(detection_graph, test_images, category_index, FLAGS.output_path)


if __name__ == '__main__':
    tf.app.run()