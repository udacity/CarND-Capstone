from styx_msgs.msg import TrafficLight
from threading import Thread
import cv2
import numpy as np
import os.path
import rospy
import six.moves.urllib as urllib
import tarfile
import tensorflow as tf

MODEL_NAME = 'faster_rcnn_resnet101_coco_11_06_2017'
MODEL_FILE = MODEL_NAME + '.tar.gz'
DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'
GRAPH_FILE = 'frozen_inference_graph.pb'
TRAFFIC_SIGNAL_CLASS = 10
LUMA_THRESHOLD = (200, 255)
MIN_LIGHT_WEIGHT = 0.1
MIN_DETECTION_SCORE = 0.1
DUMP_IMAGES = False

def get_light_state(image, signal_box):
    ''' Gets traffic light state given an image and a traffic signal box.
    Args:
        image: An image containing the traffic signal.
        signal_box: The bounding box of the traffic signal.
    Returns:
        TrafficLight
    '''
    # Get coords of the traffic signal in px.
    y_min = int(image.shape[0] * signal_box[0])
    x_min = int(image.shape[1] * signal_box[1])
    y_max = int(image.shape[0] * signal_box[2])
    x_max = int(image.shape[1] * signal_box[3])
    signal_img = image[y_min:y_max,x_min:x_max]
    if DUMP_IMAGES:
        cv2.imwrite("signal.png", signal_img)

    # Convert signal image to luma-only and HSV.
    signal_hsv = cv2.cvtColor(signal_img, cv2.COLOR_BGR2HSV)
    signal_luma = cv2.cvtColor(signal_img, cv2.COLOR_BGR2GRAY)

    # Get coords of the lights in px. The offset defines the skipped frame
    # around each light for two purposes:
    # 1) Discarting outer parts of the bounding box which can contain objects
    # not beloging to the traffic signal, e.g. sky, trees, etc.
    # 2) Discarding parts of an adjacent traffic light appearing in a wrong
    # third of the signal image.
    # Y-offset is 1/20 of the signal height. There are 6 offset parts in the
    # signal image: 2 in each light box.
    # X-offset is 1/7 of the signal height. There are 2 offset parts in the
    # signal image: one on each side.
    offset_y = int(signal_img.shape[0] / 20)
    offset_x = int(signal_img.shape[1] / 7)
    one_third = int(signal_img.shape[0] / 3)
    two_thirds = int(signal_img.shape[0] * 2 / 3)
    y_min_red = offset_y
    y_max_red = one_third - offset_y
    y_min_yellow = one_third + offset_y
    y_max_yellow = two_thirds - offset_y
    y_min_green = two_thirds + offset_y
    y_max_green = signal_img.shape[0] - offset_y
    x_min_sig = offset_x
    x_max_sig = signal_img.shape[1] - offset_x

    # Compute weight of the red light.
    red_luma = signal_luma[y_min_red:y_max_red, x_min_sig:x_max_sig]
    if DUMP_IMAGES:
        cv2.imwrite("red_luma.png", cv2.cvtColor(red_luma, cv2.COLOR_GRAY2RGB))
    red_binary = np.zeros(red_luma.shape, dtype=np.uint8)
    red_binary[(red_luma >= LUMA_THRESHOLD[0]) \
        & (red_luma <= LUMA_THRESHOLD[1])] = 1
    red_hsv = signal_hsv[y_min_red:y_max_red, x_min_sig:x_max_sig]
    red_mask1 = cv2.inRange(red_hsv,
                            np.array([0, 100, 100]),
                            np.array([10, 255, 255]))
    red_mask2 = cv2.inRange(red_hsv,
                            np.array([170, 100, 100]),
                            np.array([180, 255, 255]))
    red_mask = np.clip(
        red_mask1.astype(np.uint16) + red_mask2.astype(np.uint16),
        0, 255).astype(np.uint8)
    if DUMP_IMAGES:
        cv2.imwrite("red_mask.png", cv2.cvtColor(red_mask, cv2.COLOR_GRAY2RGB))
    red_binary += np.clip(red_mask, 0, 1)
    red_binary = np.clip(red_binary, 0, 1)
    if DUMP_IMAGES:
        cv2.imwrite("red_binary.png",
                    cv2.cvtColor(red_binary * 255, cv2.COLOR_GRAY2RGB))
    red_weight = np.average(red_binary)

    # Compute weight of the yellow light.
    yellow_luma = signal_luma[y_min_yellow:y_max_yellow, x_min_sig:x_max_sig]
    if DUMP_IMAGES:
        cv2.imwrite("yellow_luma.png",
                    cv2.cvtColor(yellow_luma, cv2.COLOR_GRAY2RGB))
    yellow_binary = np.zeros(yellow_luma.shape, dtype=np.uint8)
    yellow_binary[(yellow_luma >= LUMA_THRESHOLD[0]) \
        & (yellow_luma <= LUMA_THRESHOLD[1])] = 1
    yellow_hsv = signal_hsv[y_min_yellow:y_max_yellow, x_min_sig:x_max_sig]
    yellow_mask = cv2.inRange(yellow_hsv,
                              np.array([20, 100, 100]),
                              np.array([30, 255, 255]))
    if DUMP_IMAGES:
        cv2.imwrite("yellow_mask.png",
                    cv2.cvtColor(yellow_mask, cv2.COLOR_GRAY2RGB))
    yellow_binary += np.clip(yellow_mask, 0, 1)
    yellow_binary = np.clip(yellow_binary, 0, 1)
    if DUMP_IMAGES:
        cv2.imwrite("yellow_binary.png",
                    cv2.cvtColor(yellow_binary * 255, cv2.COLOR_GRAY2RGB))
    yellow_weight = np.average(yellow_binary)

    # Compute weight of the green light.
    green_luma = signal_luma[y_min_green:y_max_green, x_min_sig:x_max_sig]
    if DUMP_IMAGES:
        cv2.imwrite("green_luma.png",
                    cv2.cvtColor(green_luma, cv2.COLOR_GRAY2RGB))
    green_binary = np.zeros(green_luma.shape, dtype=np.uint8)
    green_binary[(green_luma >= LUMA_THRESHOLD[0]) \
        & (green_luma <= LUMA_THRESHOLD[1])] = 1
    green_hsv = signal_hsv[y_min_green:y_max_green, x_min_sig:x_max_sig]
    green_mask = cv2.inRange(green_hsv,
                             np.array([50, 100, 100]),
                             np.array([70, 255, 255]))
    if DUMP_IMAGES:
        cv2.imwrite("green_mask.png",
                    cv2.cvtColor(green_mask, cv2.COLOR_GRAY2RGB))
    green_binary += np.clip(green_mask, 0, 1)
    green_binary = np.clip(green_binary, 0, 1)
    if DUMP_IMAGES:
        cv2.imwrite("green_binary.png",
                    cv2.cvtColor(green_binary * 255, cv2.COLOR_GRAY2RGB))
    green_weight = np.average(green_binary)

    # Classify lights.
    lights = [(TrafficLight.RED, red_weight),
              (TrafficLight.YELLOW, yellow_weight),
              (TrafficLight.GREEN, green_weight)]
    lights.sort(key=lambda tup: tup[1], reverse=True)
    rospy.loginfo("Lights %s", lights)
    if lights[0][1] > MIN_LIGHT_WEIGHT and lights[1][1] / lights[0][1] < 0.5:
        # Disregard the light whose average weight is too low, or
        # whose weight is less than twice higher than the second
        # greatest one.
        return lights[0][0]
    return TrafficLight.UNKNOWN

class TLClassifier(Thread):
    def __init__(self, queue, classified_cb):
        ''' Constructor.
        Args:
            queue: Task queue. Each task is a tuple (stop_wp, image).
            classified_cb: Completion callback.
        '''
        Thread.__init__(self)
        self.queue = queue
        self.classified_cb = classified_cb

    def run(self):
        ''' Overriden method Thread::run() representing the thread's activity.
        '''
        if not os.path.isfile(GRAPH_FILE):
            rospy.loginfo("Downloading Tensorflow model")
            # Download Model.
            opener = urllib.request.URLopener()
            opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
            tar_file = tarfile.open(MODEL_FILE)
            for file in tar_file.getmembers():
                file_name = os.path.basename(file.name)
                if GRAPH_FILE in file_name:
                    file.name = os.path.basename(file.name)
                    tar_file.extract(file, os.getcwd())
                    break
        rospy.loginfo("Loading Tensorflow model into memory")
        # Load a (frozen) Tensorflow model into memory.
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(GRAPH_FILE, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            # Definite input and output Tensors for detection_graph.
            image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object
            # was detected.
            detection_boxes = detection_graph.get_tensor_by_name(
                'detection_boxes:0')
            # Each score represent how level of confidence for each of the
            # objects. Score is shown on the result image, together with the
            # class label.
            detection_scores = detection_graph.get_tensor_by_name(
                'detection_scores:0')
            detection_classes = detection_graph.get_tensor_by_name(
                'detection_classes:0')
            num_detections = detection_graph.get_tensor_by_name(
                'num_detections:0')
            session = tf.Session(graph=detection_graph)
            # Classify a dummy image to launch the pipeline.
            dummy_image = np.zeros((1, 1, 1, 3), dtype=np.uint8)
            (_, _, _, _) = session.run(
                [detection_boxes,
                 detection_scores,
                 detection_classes,
                 num_detections],
                feed_dict={image_tensor: dummy_image})
        rospy.loginfo("Classifier has been initialized")

        while True:
            # Read an image from the queue and determine the color of the
            # traffic light. Queue:get() is a blocking call.
            stop_wp, image = self.queue.get()
            # Expand dimensions since the model expects images to have shape:
            # [1, None, None, 3]
            image_expanded = np.expand_dims(image, axis=0)
            # Actual detection.
            begin_time = rospy.get_rostime()
            (boxes, scores, classes, num) = session.run(
                [detection_boxes,
                 detection_scores,
                 detection_classes,
                 num_detections],
                feed_dict={image_tensor: image_expanded})
            end_time = rospy.get_rostime()
            rospy.logdebug("Inference duration %.3f s",
                           end_time.to_sec() - begin_time.to_sec())
            highest_score = 0.
            signal_detected = False
            # Iterate through detections and find the traffic signal of highest
            # probability.
            for n in range(num):
                current_class = np.squeeze(classes)[n]
                current_score = np.squeeze(scores)[n]
                if current_class == TRAFFIC_SIGNAL_CLASS \
                        and current_score > highest_score:
                    signal_box = np.squeeze(boxes)[n]
                    highest_score = current_score
                    signal_detected = True

            # Default traffic light state is unknown.
            light_state = TrafficLight.UNKNOWN
            if signal_detected and highest_score > MIN_DETECTION_SCORE:
                rospy.logdebug("Signal detected with score %.3f", highest_score)
                light_state = get_light_state(image, signal_box)
            self.classified_cb(stop_wp, light_state)
            self.queue.task_done()
