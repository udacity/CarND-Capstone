import numpy as np
import os
import six.moves.urllib as urllib
import tarfile
import tensorflow as tf
from matplotlib import pyplot as plt
from PIL import Image
import cv2
from os import path
from utils import label_map_util
from utils import visualization_utils as vis_util
import time
import cv2
from tl_classifier import TLClassifier

def detect_color(image, boxes, scores, classes, max_boxes_to_draw=20, min_score_thresh=0.85, traffic_ligth_label=10):
    im_width = image.shape[1]
    im_height = image.shape[0]

    imgs_crops = []
    for i in range(min(max_boxes_to_draw, boxes.shape[0])):
        if scores[i] > min_score_thresh and classes[i] == traffic_ligth_label:
            ymin, xmin, ymax, xmax = tuple(boxes[i].tolist())

            x1 = int(xmin * im_width)
            x2 = int(xmax * im_width)
            y1 = int(ymin * im_height)
            y2 = int(ymax * im_height)

            crop_img = image[y1:y2, x1:x2]
            imgs_crops.append(crop_img)

    if(len(imgs_crops) > 1):
        state = clf.get_classification_batch_argmax(image_list=imgs_crops)
    elif(len(imgs_crops) == 1):
        state = clf.get_classification(image=imgs_crops[0])
    else:
        state = -1

    return state

def plot_origin_image(image_np, boxes, classes, scores, category_index):

    # Size of the output images.
    IMAGE_SIZE = (12, 8)
    vis_util.visualize_boxes_and_labels_on_image_array(
      image_np,
      np.squeeze(boxes),
      np.squeeze(classes).astype(np.int32),
      np.squeeze(scores),
      category_index,
      min_score_thresh=.5,
      use_normalized_coordinates=True,
      line_thickness=3)
    plt.figure(figsize=IMAGE_SIZE)
    plt.imshow(image_np)

    # save augmented images into hard drive
    # plt.savefig( 'output_images/ouput_' + str(idx) +'.png')
    plt.show()


def detect_traffic_lights(PATH_TO_TEST_IMAGES_DIR, MODEL_NAME, Num_images, plot_flag=False):

    TEST_IMAGE_PATHS = [ os.path.join(PATH_TO_TEST_IMAGES_DIR, 'img_{}.jpg'.format(i)) for i in range(1, Num_images+1) ]

    traffic_light_states = []

    MODEL_FILE = MODEL_NAME + '.tar.gz'
    DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'

    # Path to frozen detection graph. This is the actual model that is used for the object detection.
    PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

    # List of the strings that is used to add correct label for each box.
    PATH_TO_LABELS = 'mscoco_label_map.pbtxt'

    # number of classes for COCO dataset
    NUM_CLASSES = 90


    if path.isdir(MODEL_NAME) is False:
        opener = urllib.request.URLopener()
        opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
        tar_file = tarfile.open(MODEL_FILE)
        for file in tar_file.getmembers():
          file_name = os.path.basename(file.name)
          if 'frozen_inference_graph.pb' in file_name:
            tar_file.extract(file, os.getcwd())

    detection_graph = tf.Graph()
    with detection_graph.as_default():
      od_graph_def = tf.GraphDef()
      with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')


    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map,
                                                                max_num_classes=NUM_CLASSES,
                                                                use_display_name=True)
    category_index = label_map_util.create_category_index(categories)


    with detection_graph.as_default():
        with tf.Session(graph=detection_graph) as sess:

            # Definite input and output Tensors for detection_graph
            # Each box represents a part of the image where a particular object was detected.
            # return the level of confidence for each of the box detects.

            image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
            detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
            detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
            detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = detection_graph.get_tensor_by_name('num_detections:0')

            for image_path in TEST_IMAGE_PATHS:
                image = cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB)

                # reshape to [1, None, None, 3]
                image_np_expanded = np.expand_dims(image, axis=0)
                # Detect
                (boxes, scores, classes, num) = sess.run(
                  [detection_boxes, detection_scores, detection_classes, num_detections],
                  feed_dict={image_tensor: image_np_expanded})


                state = detect_color(image, np.squeeze(boxes), np.squeeze(scores), np.squeeze(classes).astype(np.int32))

                # if (state == 0):
                #     print('RED', state)
                # elif (state == 1):
                #     print('YELLOW', state)
                # elif (state == 2):
                #     print('GREEN', state)
                # else:
                #     print('No traffic light detected', state)

                traffic_light_states.append(state)
                # Visualization of the results of a detection.
                if plot_flag:
                    plot_origin_image(image, boxes, classes, scores, category_index)

    return traffic_light_states

if __name__ == "__main__":
    clf = TLClassifier("gc_classifier_v1_p27_est.pkl")
    Num_images = 18
    PATH_TO_TEST_IMAGES_DIR = './test_images'
    MODEL_NAME = 'faster_rcnn_resnet101_coco_11_06_2017'

    tls = detect_traffic_lights(PATH_TO_TEST_IMAGES_DIR, MODEL_NAME, Num_images, plot_flag=False)

    print(tls)
