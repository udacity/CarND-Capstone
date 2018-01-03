import os
import sys
import tensorflow as tf
import numpy as np
import cv2
import matplotlib.pyplot as plt
import argparse
import time
from object_detection.utils import object_detection_evaluation
from object_detection.utils import label_map_util
from object_detection.core.standard_fields import DetectionResultFields, InputDataFields

plt.style.use('ggplot')

PATH_TEST_IMAGES = 'test_images'
TIMING_ANALYSIS_RUNS = 10


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


def draw_gt_bounding_boxes(image, class_labels, class_text, boxes, line_thickness=4):
    """ Draws a bounding box.

    :param image:            RGB numpy image.
    :param boxes:            List of bounding boxes.
    :param scores:           Detection scores.
    :param min_score_thresh: Minimum score threshold for a box to be visualized. If None draw all boxes.
    :param line_thickness:   Thickness of bounding box lines.
    :param classes:          Traffic light classes (1=undefined, 2=red, 3=yellow, 4=green)

    :return: Image with bounding box overlay.
    """
    height, width, _ = image.shape

    for i in range(len(class_labels)):
        color = (100, 100, 100)
        text = 'GT_undefined'

        if class_labels[i] == 2:
            color = (150, 0, 0)
            text = 'GT_red'
        elif class_labels[i] == 3:
            color = (150, 150, 0)
            text = 'GT_yellow'
        elif class_labels[i] == 4:
            color = (0, 150, 0)
            text = 'GT_green'

        x_max = int(round(boxes[1][i] * width))
        x_min = int(round(boxes[0][i] * width))
        y_max = int(round(boxes[3][i] * height))
        y_min = int(round(boxes[2][i] * height))

        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, thickness=line_thickness)
        cv2.putText(image, text,
                    (x_min, y_max + line_thickness + 10),
                    cv2.FONT_HERSHEY_DUPLEX,
                    fontScale=0.4,
                    color=color,
                    lineType=1)
    return image


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


def run_tl_detector_on_test_images(min_score_thresh=0.5, waitkey=False):
    """ Run TL model on test images.

    :param min_score_thresh: Minimum score threshold for a box to be visualized. If None draw all boxes.
    :param waitkey:          If true, wait for keyboard input after each TL detection.
    """
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
                print('Run TL detecton on {}'.format(image_path))
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

                image = draw_bounding_boxes(image, boxes, scores, classes,
                                            min_score_thresh=min_score_thresh,
                                            line_thickness=2)
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                cv2.imshow('Traffic Light Detection', image)
                print('Press any key to continue.')
                wait = 0 if waitkey else 1

                if cv2.waitKey(wait) & 0xFF == ord('q'):
                    break


def evaluate_model(tfrecord_files, waitkey=False):
    """ Applies the PASACAL measurement on the test dataset.

    :param tfrecord_files:  List of TFRecord files.
    :param waitkey:         If true, wait for keyboard input after each TL detection.
    """

    label_map_path = 'tl_model_config/traffic_light_label_map.pbtxt'

    feature_set = {'image/height': tf.FixedLenFeature([], tf.int64),
                   'image/width': tf.FixedLenFeature([], tf.int64),
                   'image/filename': tf.FixedLenFeature([], tf.string),
                   'image/source_id': tf.FixedLenFeature([], tf.string),
                   'image/encoded': tf.FixedLenFeature([], tf.string),
                   'image/format': tf.FixedLenFeature([], tf.string),
                   'image/object/bbox/xmin': tf.VarLenFeature(tf.float32),
                   'image/object/bbox/xmax': tf.VarLenFeature(tf.float32),
                   'image/object/bbox/ymin': tf.VarLenFeature(tf.float32),
                   'image/object/bbox/ymax': tf.VarLenFeature(tf.float32),
                   'image/object/class/text': tf.VarLenFeature(tf.string),
                   'image/object/class/label': tf.VarLenFeature(tf.int64)}

    filename_queue = tf.train.string_input_producer(tfrecord_files, num_epochs=None)
    reader = tf.TFRecordReader()
    _, serialized_example = reader.read(filename_queue)
    features = tf.parse_single_example(serialized_example, features=feature_set)

    f_width = features['image/width']
    f_height = features['image/height']
    f_filename = features['image/filename']
    f_source_id = features['image/source_id']
    f_encoded_image = features['image/encoded']
    f_format = features['image/format']
    f_xmin = features['image/object/bbox/xmin']
    f_xmax = features['image/object/bbox/xmax']
    f_ymin = features['image/object/bbox/ymin']
    f_ymax = features['image/object/bbox/ymax']
    f_class_text = features['image/object/class/text']
    f_class_label = features['image/object/class/label']

    sess = tf.Session()
    init = tf.global_variables_initializer()
    sess.run(init)
    tf.train.start_queue_runners(sess=sess)

    # Define input and output Tensors for detection_graph
    tl_graph = detection_graph.as_default()
    tl_sess = tf.Session(graph=detection_graph)
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
    detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    # Load label map and initialize PASCAL evaluator
    label_map = label_map_util.load_labelmap(label_map_path)
    max_num_classes = max([item.id for item in label_map.item])
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes)
    evaluator = object_detection_evaluation.PascalDetectionEvaluator(categories)

    num_images = 0

    while 1:
        # load next image from TFRecord file
        width, height, filename, source_id, encoded_image, format, x_min, x_max, y_min, y_max, class_text, class_label\
            = sess.run([f_width, f_height, f_filename, f_source_id, f_encoded_image, f_format,
                        f_xmin, f_xmax, f_ymin, f_ymax,
                        f_class_text, f_class_label])

        #print('w={} h={} {} {}'.format(width, height, filename, format))
        #print(x_min.values * width)
        #print(x_max.values * width)
        #print(y_min.values * height)
        #print(y_max.values * height)
        #print(class_text.values)
        #print(class_label.values)

        if format is b'jpeg':
            image_tf = tf.image.decode_jpeg(encoded_image, channels=3)
        else:
            image_tf = tf.image.decode_png(encoded_image, channels=3)

        image_tf = tf.reshape(image_tf, [height, width, 3])
        image = sess.run(image_tf)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_expanded = np.expand_dims(image, axis=0)

        # Detect traffic lights
        (boxes, scores, classes, num) = tl_sess.run([detection_boxes,
                                                     detection_scores,
                                                     detection_classes,
                                                     num_detections],
                                                    feed_dict={image_tensor: image_expanded})

        num_images += 1

        #print('Number of detections: {}'.format(num))
        #print('bounding boxes:')
        #print(boxes)
        #print('classes:')
        #print(classes)
        #print('scores:')
        #print(scores)

        # Show reaults in image
        image = draw_gt_bounding_boxes(image, class_label.values, class_text.values,
                                       [x_min.values, x_max.values, y_min.values, y_max.values],
                                       line_thickness=1)
        image = draw_bounding_boxes(image, boxes, scores, classes, min_score_thresh=0.5, line_thickness=2)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imshow('Traffic Light Evaluation', image)

        print('--------------------------------------------------------------------------------------')
        print('TL-Model Performance (num_images={:05d}):'.format(num_images))
        print('GT TLs / Detected TLs: {:2d}/{:2d}'.format(len(class_label.values), len(np.where(scores >= 0.5)[0])))

        # Add groundtruth to evaluator
        groundtruth_boxes = []
        groundtruth_classes = []

        for i in range(len(class_label.values)):
            groundtruth_boxes.append([y_min.values[i] * height, x_min.values[i] * width, y_max.values[i] * height, x_max.values[i] * width])
            groundtruth_classes.append(class_label.values[i])

        if len(groundtruth_classes) > 0 or len(scores[0]) > 0:
            groundtruth_dict = {InputDataFields.groundtruth_boxes: np.array(groundtruth_boxes, dtype=np.float32),
                                InputDataFields.groundtruth_classes: np.array(groundtruth_classes)}
            evaluator.add_single_ground_truth_image_info(num_images, groundtruth_dict)

            # Scale detection results to absolute coordinates and add them to evaluator
            boxes_absolute = np.squeeze(boxes, axis=0)
            scores = np.squeeze(scores, axis=0)
            classes = np.squeeze(classes, axis=0)

            for i in range(len(boxes_absolute)):
                boxes_absolute[i][0] *= height
                boxes_absolute[i][1] *= width
                boxes_absolute[i][2] *= height
                boxes_absolute[i][3] *= width

            detections_dict = {DetectionResultFields.detection_boxes: boxes_absolute,
                               DetectionResultFields.detection_scores: scores,
                               DetectionResultFields.detection_classes: classes}
            evaluator.add_single_detected_image_info(num_images, detections_dict)

            metrics = evaluator.evaluate()

            for key in metrics:
                print('{:52s}: {:.4f}'.format(key, metrics[key]))

        # Press 'q' to quit
        print('Press q to quit')
        wait = 0 if waitkey else 1

        if cv2.waitKey(wait) & 0xFF == ord('q'):
            break


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
        default=False,
        dest='run_test_images'
    )

    parser.add_argument(
        '-s', '--min_score_threshold',
        help='Min score threshold to draw the bounding boxes.',
        dest='min_score_threshold',
        default=0.5,
        type=float,
        metavar='MIN_SCORE_THRESHOLD'
    )

    parser.add_argument(
        '-p', '--run_pascal',
        help='Run TL detector and calculate PASCAL values.',
        dest='run_pascal',
        metavar='TFRECORD_FILE'
    )

    parser.add_argument(
        '-w', '--waitkey',
        help='Waits for keyboard input ater each TL detection.',
        action='store_true',
        default=False,
        dest='waitkey'
    )

    parser.add_argument(
        '-n', '--show_graph_node_names',
        help='Shows all graph node names.',
        action='store_true',
        dest='show_graph_node_names',
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
            # run timing analysis for different image resolutions
            timing_analysis()
        elif args.run_test_images:
            # run frozen model on test images and show results
            run_tl_detector_on_test_images(waitkey=args.waitkey)
        elif args.run_pascal:
            # Calculate PASCAL values for given dataset
            evaluate_model(args.run_pascal.split(','), waitkey=args.waitkey)
        elif args.show_graph_node_names:
            # show all graph node names
            print('Operations in Optimized Graph:')
            for op in detection_graph.get_operations():
                print(op.name)
