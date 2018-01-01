import os
import sys
import tensorflow as tf
import numpy as np
import cv2
import matplotlib.pyplot as plt
import argparse
import time

plt.style.use('ggplot')

PATH_TEST_IMAGES = 'model/research/object_detection/test_images'
TIMING_ANALYSIS_RUNS = 2


def time_detection(sess, width, height, runs=10):
    """ Timing analysis on random images with given shape.

    :param sess:    Tensorflow session.
    :param width:   Test image width
    :param height:  Test image height
    :param runs:    Number of model runs.

    :return: Returns an array of timings.
    """
    image_tensor = sess.graph.get_tensor_by_name('image_tensor:0')
    detection_boxes = sess.graph.get_tensor_by_name('detection_boxes:0')
    detection_scores = sess.graph.get_tensor_by_name('detection_scores:0')
    detection_classes = sess.graph.get_tensor_by_name('detection_classes:0')

    # warmup
    gen_image = np.uint8(np.random.randn(1, height, width, 3))
    sess.run([detection_boxes, detection_scores, detection_classes], feed_dict={image_tensor: gen_image})

    times = np.zeros(runs)

    for i in range(runs):
        t0 = time.time()
        sess.run([detection_boxes, detection_scores, detection_classes], feed_dict={image_tensor: gen_image})
        t1 = time.time()
        times[i] = (t1 - t0) * 1000

    return times

def timing_analysis():
    """ Analysis the model timing on different image resolutions."""

    resolutions = [[ 640, 480],
                   [ 800, 600],
                   [1024, 768],
                   [1280, 720],
                   [1368, 1096]]
    timings = []
    res_text = []

    with detection_graph.as_default():
        with tf.Session(graph=detection_graph) as sess:
            for res in resolutions:
                print('Timing analysis (runs={}, res={}x{})...'.format(TIMING_ANALYSIS_RUNS, res[0], res[1]), end='', flush=True)
                times = time_detection(sess, res[0], res[1], runs=TIMING_ANALYSIS_RUNS)
                timings.append(times)
                res_text.append('{}x{}'.format(res[0], res[1]))
                print('done')

    # Show timing results
    print('Timing Results (runs={}):'.format(TIMING_ANALYSIS_RUNS))
    for i in range(len(timings)):
        t = np.array(timings[i])
        print('{:4d}x{:4d}: min={:7.2f} max={:7.2f} mean={:7.2f} avg={:7.2f} std={:7.2f}'.format(
            resolutions[i][0], resolutions[i][1],
            np.min(t), np.max(t),
            np.mean(t), np.average(t),
            np.std(t)))

    fig = plt.figure(1, figsize=(9, 6))
    ax = fig.add_subplot(111)
    plt.title("Traffic Light Detection Timings (runs={})".format(TIMING_ANALYSIS_RUNS))
    plt.ylabel("Time (ms)")
    plt.style.use('fivethirtyeight')
    bp = ax.boxplot(timings, labels=res_text, showmeans=True)
    plt.show()


def draw_bounding_boxes(image, boxes, scores, classes, min_score_thresh=0.5, line_thickness=4):
    """ Draws the bounding boxes.

    :param image:            RGB numpy image.
    :param boxes:            List of bounding boxes.
    :param scores:           Detection scores.
    :param min_score_thresh: Minimum score threshold for a box to be visualized. If None draw all boxes.
    :param line_thickness:   Thickness of bounding box lines.
    :param classes:          Traffic light classes (1=undefined, 2=red, 3=yellow, 4=green)

    :return: Image with bounding box overlay.
    """
    height, width, _ = image.shape

    for i in range(len(boxes)):
        for n in range(len(boxes[i])):
            if min_score_thresh is None or scores[i][n] >= min_score_thresh:
                color = (100, 100, 100)
                text = 'TL_undefined'

                if classes[i][n] == 2:
                    color = (255, 0, 0)
                    text = 'TL_red'
                elif classes[i][n] == 3:
                    color = (255, 255, 0)
                    text = 'TL_yellow'
                elif classes[i][n] == 4:
                    color = (0, 255, 0)
                    text = 'TL_green'

                text = '{} {:.1f}%'.format(text, scores[i][n] * 100.0)
                x_max = int(round(boxes[i][n][3] * width))
                x_min = int(round(boxes[i][n][1] * width))
                y_max = int(round(boxes[i][n][2] * height))
                y_min = int(round(boxes[i][n][0] * height))

                cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, thickness=line_thickness)
                cv2.putText(image, text,
                            (x_min, y_min - line_thickness - 1),
                            cv2.FONT_HERSHEY_DUPLEX,
                            fontScale=0.4,
                            color=color,
                            lineType=1)
    return image


def run_tl_detector_on_test_images():
    """ Run TL model on test images. """
    # load test images
    test_sets = ['real', 'sim', 'bosch']

    print('Loading test images...', end='', flush=True)
    test_image_paths = []
    for filename in os.listdir(PATH_TEST_IMAGES):
        path = os.path.join(PATH_TEST_IMAGES, filename)
        for test_set in test_sets:
            if os.path.isfile(path) and test_set in filename and ('.jpg' in filename or '.png' in filename):
                test_image_paths.append(path)
    print('done')

    # detect traffic lights
    with detection_graph.as_default():
        with tf.Session(graph=detection_graph) as sess:
            # Definite input and output Tensors for detection_graph
            image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
            detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
            detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
            detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = detection_graph.get_tensor_by_name('num_detections:0')

            for image_path in test_image_paths:
                print('Image: {}'.format(image_path))
                image = cv2.imread(image_path)
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_expanded = np.expand_dims(image, axis=0)

                (boxes, scores, classes, num) = sess.run([detection_boxes,
                                                          detection_scores,
                                                          detection_classes,
                                                          num_detections],
                                                         feed_dict={image_tensor: image_expanded})

                print('Number of detections: {}'.format(num))
                #print('bounding boxes:')
                #print(boxes)
                print('classes:')
                print(classes)
                print('scores:')
                print(scores)

                image = draw_bounding_boxes(image, boxes, scores, classes, min_score_thresh=0.2, line_thickness=2)
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                cv2.imshow('Traffic Light Detection', image)
                cv2.waitKey(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='tl_model_eval', description='TL Model Evaluation.')

    parser.add_argument(
        '-m', '--tl_model',
        help='Path to frozen TL model file.',
        dest='tl_model',
        metavar='PB_file',
        required=True
    )

    parser.add_argument(
        '-t', '--timing_analysis',
        help='Test cycle time on different image resolutions.',
        action='store_true',
        default=False,
        dest='timing_analysis'
    )

    parser.add_argument(
        '-r', '--run_test_images',
        help='Run TL detector on test images.',
        action='store_true',
        dest='run_test_images',
        default=False
    )

    args = parser.parse_args()

    if len(sys.argv) < 4:
        # no arguments found
        parser.print_usage()
        parser.exit(-1)

    if args.tl_model:
        # load frozen traffic light model checkpoint
        print('Loading model {}...'.format(args.tl_model), end='', flush=True)
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(args.tl_model, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        print('done')

        if args.timing_analysis:
            timing_analysis()
        elif args.run_test_images:
            run_tl_detector_on_test_images()
