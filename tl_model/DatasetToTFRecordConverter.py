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

    def __init__(self, width, height, number_images_to_generate=None, train_ratio=1.0, augmentation_rate=0.0):
        """ Initialize the dataset handler with datasets.

        :param width:             Target image width.
        :param height:            Target image height.
        :param number_images_to_generate: If None, the generator generates as much images as stored in the datasets. Otherwise
                                  the specified number of images.
                                  Number of images = train set + test set
        :param train_ratio:       Size of the training dataset [%].
        :param augmentation_rate: Rate (0..1) of total images which will be randomly augmented.
                                  E.g. 0.6 augments 60% of the images and 40% are raw images
        """
        self.width = width
        self.height = height
        self.number_images_to_generate = number_images_to_generate
        self.train_ratio = train_ratio
        self.augmentation_rate = augmentation_rate
        self.temp_converted_image_name = 'tmp_converted_image'

        # setup dataset handler
        self.dataset_handler = dh.DatasetHandler(width=width, height=height)
        self.dataset_handler.read_predefined_dataset()
        #TODO: self.dataset_handler.read_all_capstone_labels('datasets/dataset_sdcnd_capstone/real_training_data/real_data_annotations.yaml')

        stop_at_end = (number_images_to_generate is None)

        if train_ratio < 1.0:
            self.train_samples, self.test_samples = self.dataset_handler.split_dataset(train_ratio=train_ratio)

            self.test_generator = self.dataset_handler.generator(self.test_samples,
                                                                 batch_size=1,
                                                                 augmentation_rate=augmentation_rate,
                                                                 bbox=True,
                                                                 stop_at_end=stop_at_end)
        else:
            self.train_samples = self.dataset_handler.samples
            self.test_samples = None
            self.test_generator = None

        self.train_generator = self.dataset_handler.generator(self.train_samples,
                                                              batch_size=1,
                                                              augmentation_rate=augmentation_rate,
                                                              bbox=True,
                                                              stop_at_end=stop_at_end)

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

    def convert_nparray_to_binary_image(self, image, format):
        """ Converts a numpy array image to the TFRecords compatible binary image.

        :param image:  Numpy binary image (e.g. read by OpenCv).
        :param format: Image format 'jpeg' or 'png'.

        :return: Returns a TFRecords compatible binary image.
        """
        if format in b'jpeg':
            filename = self.temp_converted_image_name + '.jpg'
        else:
            filename = self.temp_converted_image_name + '.png'

        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imwrite(filename, image)
        image_binary = self.load_image_binary(filename)

        return image_binary

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

        if self.augmentation_rate > 0.0 or self.width is not None or self.height is not None:
            encoded_image_data = self.convert_nparray_to_binary_image(image, image_format)
        else:
            encoded_image_data = self.load_image_binary(sample['path'])

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

    def cleanup(self):
        """ Clean-up temporary stored files. """
        filename_jpg = self.temp_converted_image_name + '.jpg'
        filename_png = self.temp_converted_image_name + '.png'

        if os.path.isfile(filename_jpg):
            os.remove(filename_jpg)

        if os.path.isfile(filename_png):
            os.remove(filename_png)

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
        dest='number_images',
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
                                           augmentation_rate=args.augmentation_rate,
                                           number_images_to_generate=args.number_images)

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

                    if converter.number_images_to_generate is not None:
                        max_number_train_images = int(round(float(converter.number_images_to_generate) * converter.train_ratio))
                        if number_train_images >= max_number_train_images:
                            break


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

                if converter.number_images_to_generate is not None:
                    max_number_test_images = int(round(float(converter.number_images_to_generate) * (1.0 - converter.train_ratio)))
                    if number_test_images >= max_number_test_images:
                        break

        print('done')
        print('Converted {} images to {}'.format(number_test_images, args.test_output_file))
        writer_test.close()

    converter.cleanup()