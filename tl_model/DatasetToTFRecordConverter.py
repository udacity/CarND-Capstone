import os
import io
import sys
import argparse
import cv2
import numpy as np
import DatasetHandler as dh
import tensorflow as tf
from model.research.object_detection.utils import dataset_util


class DatasetToTFRecordConverter:
    """ Converts the dataset created by the `DatasetHandler()` to the TFRecord format.

    The converter scripts are copied and modified from the official Tensorflow Object Detection API documentation:

        https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/using_your_own_dataset.md
    """

    def __init__(self, width, height, train_ratio=1.0, augmentation_rate=0.0):
        """ Initialize the dataset handler with datasets.

        :param width:             Target image width.
        :param height:            Target image height.
        :param train_ratio:       Size of the training dataset [%].
        :param augmentation_rate: Rate (0..1) of total images which will be randomly augmented.
                                  E.g. 0.6 augments 60% of the images and 40% are raw images
        """
        self.dataset_handler = dh.DatasetHandler(width=width, height=height)
        self.dataset_handler.read_predefined_dataset()
        # TODO: self.dataset_handler.read_all_capstone_labels('datasets/dataset_sdcnd_capstone/real_training_data/real_data_annotations.yaml')

        if train_ratio < 1.0:
            self.train_samples, self.test_samples = self.dataset_handler.split_dataset(train_ratio=train_ratio)

            self.test_generator = self.dataset_handler.generator(self.test_samples,
                                                                 batch_size=1,
                                                                 augmentation_rate=augmentation_rate,
                                                                 bbox=True,
                                                                 stop_at_end=True)
        else:
            self.train_samples = self.dataset_handler.samples
            self.test_samples = None
            self.test_generator = None

        self.train_generator = self.dataset_handler.generator(self.train_samples,
                                                              batch_size=1,
                                                              augmentation_rate=augmentation_rate,
                                                              bbox=True,
                                                              stop_at_end=True)

    def decode_image_format(self, path):
        """ Decodes TFRecord specific the image format. Only valid for jpg and png files!

        :param path: Path to the jpg or png image.

        :return: Returns the TFRecord image format.
        """
        ext = os.path.splitext(path)

        if ext[1] in '.jpg':
            return b'jpeg'
        elif ext[1] in '.png':
            return b'png'

    def load_image_binary(self, path):
        """ Load image in TFRecord format.

        :param path: Path to the jpg or png image.

        :return: Returns the TFRecord encoded image data.
        """
        with tf.gfile.GFile(path, 'rb') as fid:
            image = fid.read()
        return image

    def convert_annotations_to_lists(self, annotations, width, height):
        """ Converts the annotations (classes and bounding boxes) to separate lists. The bounding boxes are normalized
        by the image width and height.

        :param annotations: List of annotations.
        :param width:       Image width.
        :param height:      Image height.

        :return: Returns separate lists for x_mins[], x_maxs[], y_mins[], y_maxs[], classes_id[] and classes_text[].
        """
        x_mins = []
        x_maxs = []
        y_mins = []
        y_maxs = []
        classes_text = []
        classes_id = []

        for annotation in annotations:
            x_maxs.append(annotation['x_max'] / width)
            x_mins.append(annotation['x_min'] / width)
            y_maxs.append(annotation['y_max'] / height)
            y_mins.append(annotation['y_min'] / height)
            classes_id.append(annotation['class_id'])
            classes_text.append(annotation['class_text'].encode('utf8'))

        return x_mins, x_maxs, y_mins, y_maxs, classes_id, classes_text

    def create_traffic_light_tf_example(self, image, sample):
        """ Creates a tf.Example proto from sample traffic light image.

        :param image:   RGB image.
        :param sample:  Description of the sample image like
                          - path
                          - width
                          - height
                          - annotations (list of bounding boxes and classes)

        :return: example: The created tf.Example.
        """

        height = sample['height']
        width = sample['width']
        filename = sample['path'].encode('utf8')
        image_format = self.decode_image_format(sample['path'])
        encoded_image_data = self.load_image_binary(sample['path']) #FIXME: image.tobytes()

        x_mins, x_maxs, y_mins, y_maxs, classes_id, classes_text = \
            self.convert_annotations_to_lists(sample['annotations'], width=width, height=height)

        tf_example = tf.train.Example(features=tf.train.Features(feature={
            'image/height': dataset_util.int64_feature(height),
            'image/width': dataset_util.int64_feature(width),
            'image/filename': dataset_util.bytes_feature(filename),
            'image/source_id': dataset_util.bytes_feature(filename),
            'image/encoded': dataset_util.bytes_feature(encoded_image_data),
            'image/format': dataset_util.bytes_feature(image_format),
            'image/object/bbox/xmin': dataset_util.float_list_feature(x_mins),
            'image/object/bbox/xmax': dataset_util.float_list_feature(x_maxs),
            'image/object/bbox/ymin': dataset_util.float_list_feature(y_mins),
            'image/object/bbox/ymax': dataset_util.float_list_feature(y_maxs),
            'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
            'image/object/class/label': dataset_util.int64_list_feature(classes_id),
        }))
        return tf_example


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dataset to TFRecord Converter.')

    parser.add_argument(
        '--train_output_file',
        help='Path to train TFRecord output file.',
        dest='train_output_file',
        metavar='TRAIN_FILE.record'
    )

    parser.add_argument(
        '--test_output_file',
        help='Path to test TFRecord output file.',
        dest='test_output_file',
        metavar='TEST_FILE.record'
    )

    parser.add_argument(
        '--number_images',
        help='Total number of images train + test set.',
        dest='train_size',
        default=None,
        metavar='SIZE'
    )

    parser.add_argument(
        '--train_ratio',
        help='Size of the training dataset [0..1].',
        dest='train_ratio',
        default=1.0,
        type=float,
        metavar='RATIO'
    )

    parser.add_argument(
        '--augmentation_rate',
        help='Augmentation rate [0..1].',
        dest='augmentation_rate',
        default=0.65,
        type=float,
        metavar='SIZE'
    )

    args = parser.parse_args()

    if len(sys.argv) < 2:
        # no arguments found
        parser.print_usage()
        parser.exit(-1)

    # Use source image size
    converter = DatasetToTFRecordConverter(width=None,
                                           height=None,
                                           train_ratio=args.train_ratio,
                                           augmentation_rate=args.augmentation_rate)

    if not converter.dataset_handler.is_valid():
        print('ERROR: No valid datasets found.')
        exit(-1)

    number_train_images = 0
    number_test_images = 0

    if args.train_output_file:
        print('Converting train set...', end='', flush=True)
        writer_train = tf.python_io.TFRecordWriter(args.train_output_file)

        for x_train, y_train in converter.train_generator:
                if len(x_train) > 0 and len(y_train) > 0:
                    tf_example = converter.create_traffic_light_tf_example(x_train[0], y_train[0])
                    writer_train.write(tf_example.SerializeToString())
                    number_train_images += 1

        print('done')
        print('Converted {} images to {}'.format(number_train_images, args.train_output_file))
        writer_train.close()

    if args.test_output_file:
        print('Converting test set...', end='', flush=True)
        writer_test = tf.python_io.TFRecordWriter(args.test_output_file)

        for x_test, y_test in converter.test_generator:
            if len(x_test) > 0 and len(y_test) > 0:
                tf_example = converter.create_traffic_light_tf_example(x_test[0], y_test[0])
                writer_test.write(tf_example.SerializeToString())
                number_test_images += 1

        print('done')
        print('Converted {} images to {}'.format(number_test_images, args.test_output_file))
        writer_test.close()
