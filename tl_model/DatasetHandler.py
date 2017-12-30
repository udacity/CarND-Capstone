#!/usr/bin/env python
import os
import sys
import argparse
import copy
import yaml
import pandas as pd

if sys.platform.startswith('darwin'):
    import matplotlib as mpl
    mpl.use('macosx', force=True)

import matplotlib.pyplot as plt
import numpy as np
import cv2
import plot_utils as plu
import DataAugmentation as da

from enum import Enum
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split
from PIL import Image


# GT color definitions
GT_TL_RED       = [255, 0, 0]
GT_TL_YELLOW    = [255, 255, 0]
GT_TL_GREEN     = [0, 255, 0]
GT_TL_UNDEFINED = [100, 100, 100]

# Dataset annotation files
DATASET_BOSCH         = 'datasets/dataset_bosch_small_tlr/dataset_train_rgb/train.yaml'
DATASET_LARA          = 'datasets/dataset_lara/Lara_UrbanSeq1_GroundTruth_GT.txt'
DATASET_CAPSTONE_REAL = 'datasets/dataset_sdcnd_capstone/real_training_data/real_data_annotations.yaml'
DATASET_CAPSTONE_SIM  = 'datasets/dataset_sdcnd_capstone/sim_training_data/sim_data_annotations.yaml'


class DatasetType(Enum):
    """ Available datasets. """
    NONE = 0                # Empty dataset
    MERGED = 1              # Merged different datasets
    BOSCH = 2               # Bosch small traffic light dataset
    LARA = 3                # LARA traffic light dataset (La Route Automatisée, Paris)
    SDCND_CAPSTONE = 4      # Udacity specific dataset


class DatasetHandler:
    """
    The DatasetHandler provides basic methods to translate the different dataset ground truth data into
    one common format which can be used to train the TL model.

    The Bosch related statistics and labels scripts are inspired by the original Bosch GitHub repository:

        https://github.com/bosch-ros-pkg/bstld

    Usage: DatasetHandler.py -h] [-h] [--bosch_label_file YAML_file]
                                      [--lara_label_file XML_file]
                                      [--capstone_label_file YAML_file]
                                      [-df] [-s] [-sp] [-si] [-sg]

        -h, --help                       Show this help message and exit
        --bosch_label_file YAML_file     Path to the Bosch label YAML file.
        --lara_label_file XML_file       Path to the LARA label XML file.
        --capstone_label_file YAML_file  Path to the SDCND Capstone label YAML file.
        -df, --disable_filter            If set, all labels will be added to the merged dataset. Otherwise red, yellow, green, off only.
        -s, --statistics                 Show dataset statistics like label distribution.
        -sp, --safe_plots                Safe plots as PNG files.
        -si, --show_images               Show all labeled images in the dataset in a video.
        -sg, --show_generator            Show generator images and labels in a video.

    Common `label_dict Format:

    samples<list>
      |-- [0]<dict>
      |   |-- 'annotations'<list>
      |   |    |-- [0]<dict>
      |   |    |   |-- 'class_id' : <int>
      |   |    |   |-- 'class_text' : <str>
      |   |    |   |-- 'x_max! : <float>
      |   |    |   |-- 'x_min! : <float>
      |   |    |   |-- 'y_max! : <float>
      |   |    |   |-- 'y_min! : <float>
      |   |    `-- [1]<dict>
      |   |    |   |-- 'class_id' : <int>
      |   |    |   |-- 'class_text' : <str>
      |   |        |-- 'x_max! : <float>
      |   |        |-- 'x_min! : <float>
      |   |        |-- 'y_max! : <float>
      |   |        |-- 'y_min! : <float>
      |   |-- 'path': <str>
      |   |-- 'height': <int>
      |   `-- 'width': <int>
      |-- [1]<dict>
      |   |-- 'annotations'<list>
      |   |-- 'path': <str>
      |   |-- 'height': <float>
      |   `-- 'width': <float>
      ...
    """

    dataset_type = DatasetType.NONE    # type of dataset
    number_datasets = 0                # Number of merged datasets
    number_samples = 0                 # number of images in the dataset
    number_labeled_traffic_lights = 0  # number of labeled traffic lights in the dataset
    samples = []                       # list of images and labeled data (annotations)
    label_statistics = {}              # dictionary of type <TL class> : <number of labeled TL class>
    image_shape = (0, 0, 0)            # image shape [height, width, channels] of dataset

    # label class id to class text mapping.
    # This line shall be consistent to the used traffic_light_label_map.pbtxt file.
    # Attention: The fist class id shall not be 0.
    class_dict = {1: 'TL_undefined',
                  2: 'TL_red',
                  3: 'TL_yellow',
                  4: 'TL_green'}

    def __init__(self, width=None, height=None, verbose=False):
        """ Initializes the DatasetHandler.

        :param width:     Width of the generator output image.
        :param height:    Height of the generator output image.
        :param verbose:   If true, the the generator visualizes its output.
        """
        self.generator_image_shape = (width, height)
        self.verbose = verbose

        if verbose:
            heatmap_shape = (self.generator_image_shape[1], self.generator_image_shape[0])
            self.generator_heatmap_red = np.zeros(heatmap_shape, dtype=np.float)
            self.generator_heatmap_yellow = np.zeros(heatmap_shape, dtype=np.float)
            self.generator_heatmap_green = np.zeros(heatmap_shape, dtype=np.float)
            self.generator_heatmap_off = np.zeros(heatmap_shape, dtype=np.float)
            self.generator_heatmap_all = np.zeros(heatmap_shape, dtype=np.float)

            plt.ion()
            self.fig_heatmap_all, self.ax_heatmap_all = plt.subplots()
            self.fig_heatmap_all.subplots_adjust(left=0.1, right=0.97, bottom=0.1, top=0.9)
            self.fig_heatmap_all.suptitle('Label Heatmap of Generator Traffic Light Dataset')
            self.plt_heatmap_all = self.ax_heatmap_all.imshow(self.generator_heatmap_all, cmap='jet', interpolation='nearest')
            self.ax_heatmap_all.set_title('Red, Green, Red, Off TL')

            self.fig_heatmap, self.axarr_heatmap = plt.subplots(2, 2)
            self.fig_heatmap.subplots_adjust(left=0.1, right=0.97, bottom=0.1, top=0.9)
            self.fig_heatmap.suptitle('Label Heatmap of Generator Traffic Light Dataset')
            self.plt_heatmap_red = self.axarr_heatmap[0, 0].imshow(self.generator_heatmap_red, cmap='Reds', interpolation='nearest')
            self.axarr_heatmap[0, 0].set_title('Red TL')
            self.plt_heatmap_yellow = self.axarr_heatmap[0, 1].imshow(self.generator_heatmap_yellow, cmap='Oranges', interpolation='nearest')
            self.axarr_heatmap[0, 1].set_title('Yellow TL')
            self.plt_heatmap_green = self.axarr_heatmap[1, 0].imshow(self.generator_heatmap_green, cmap='Greens', interpolation='nearest')
            self.axarr_heatmap[1, 0].set_title('Green TL')
            self.plt_heatmap_off = self.axarr_heatmap[1, 1].imshow(self.generator_heatmap_off, cmap='Greys', interpolation='nearest')
            self.axarr_heatmap[1, 1].set_title('Off TL')

    def update_dataset_type(self, dataset_type):
        """ Updates the dataset type. In case several dataset have been read the type is set to `DatasetType.MERGED`.

        :param type: Dataset type of enum type `DatasetType` (MERGED, BOSCH, LARA, SDCND_SCAPSTONE).
        """
        if self.number_datasets == 1:
            self.dataset_type = dataset_type
        else:
            self.dataset_type = DatasetType.MERGED

    def get_class_id_text(self, label):
        """ Determines the class id and text for the orinial label.

        :param label: Label from orinial annotation file. It shall contain red, yellow or green in
                      order to ensure a valid match.

        :return: Returns the correspondig class id and class text.
        """
        if label.lower().find('red') >= 0:
            return 2, self.class_dict[2]
        elif label.lower().find('yellow') >= 0:
            return 3, self.class_dict[3]
        elif label.lower().find('green') >= 0:
            return 4, self.class_dict[4]
        else:
            return 1, self.class_dict[1]

    def get_image_shape(self, path):
        """ Returns the image shape witout loading the image into memory.

        :param path: Path to image.

        :return: Returns the image shape (height, width, channels).
        """
        image = Image.open(path)
        return (image.height, image.width, 3)

    def read_all_bosch_labels(self, input_yaml, riib=False, filter_labels=True):
        """
        Gets all labels from the Bosch Small Traffic Light Dataset listed in the label file.

        Image Formats:
          - RIIB 12 bit (red-clear-clear-blue), size = 1280x736
          - RGB 8bit (red-green-blue), size = 1280x720, Reconstructed color images.

        The RGB images are provided for debugging and can also be used for training. However, the RGB conversion
        process has some drawbacks. Some of the converted images may contain artifacts and the color distribution
        may seem unusual.

        Bosch Label File Format:

        - boxes:
          - {label: Green, occluded: false, x_max: 752.625, x_min: 748.875, y_max: 354.25, y_min: 343.375}
          path: /net/pal-soc1.us.bosch.com/ifs/data/Shared_Exports/deep_learning_data/traffic_lights/university_run1/24070.png
        - boxes: []
          path: /net/pal-soc1.us.bosch.com/ifs/data/Shared_Exports/deep_learning_data/traffic_lights/university_run1/24460.png
        - boxes:
          - {label: Green, occluded: false, x_max: 798.875, x_min: 793.375, y_max: 373.25, y_min: 357.875}
          - {label: 'off', occluded: false, x_max: 359.875, x_min: 355.125, y_max: 383.375, y_min: 366.375}
          path: /net/pal-soc1.us.bosch.com/ifs/data/Shared_Exports/deep_learning_data/traffic_lights/university_run1/24740.png

        :param input_yaml:     Path to yaml file.
        :param riib:           If True, change path to labeled pictures.
        :param filter_labels:  If true only red, yellow, green and off labels are considered.

        :return: images: Labels for traffic lights. None in case the input file couldn't be found.
        """
        if not os.path.exists(input_yaml):
            return None

        images = yaml.load(open(input_yaml, 'rb').read())

        for i in range(len(images)):
            images[i]['path'] = os.path.abspath(os.path.join(os.path.dirname(input_yaml), images[i]['path']))

            if riib:
                images[i]['path'] = images[i]['path'].replace('.png', '.pgm')
                images[i]['path'] = images[i]['path'].replace('rgb/train', 'riib/train')
                images[i]['path'] = images[i]['path'].replace('rgb/test', 'riib/test')

                for box in images[i]['boxes']:
                    box['y_max'] = box['y_max'] + 8
                    box['y_min'] = box['y_min'] + 8

            # convert to generic label format and generate some basic dataset statistics
            annotations = []
            path = images[i]['path']

            # determine image size
            self.image_shape = self.get_image_shape(path)
            height = self.image_shape[0]
            width = self.image_shape[1]

            for box in images[i]['boxes']:
                # filter for relevant labels if activated
                if not filter_labels or \
                        ((box['label'].lower().find('red') >= 0 and len(box['label']) == len('red')) or \
                         (box['label'].lower().find('yellow') >= 0 and len(box['label']) == len('yellow')) or \
                         (box['label'].lower().find('green') >= 0 and len(box['label']) == len('green')) or \
                         (box['label'].lower().find('off') >= 0 and len(box['label']) == len('off'))):

                    class_id, class_text = self.get_class_id_text(box['label'])
                    annotation = {'class_id': class_id,
                                  'class_text': class_text,
                                  'x_max': box['x_max'],
                                  'x_min': box['x_min'],
                                  'y_max': box['y_max'],
                                  'y_min': box['y_min']}

                    annotations.append(annotation)
                    self.number_labeled_traffic_lights += 1

                    if annotation['class_text'] not in self.label_statistics.keys():
                        self.label_statistics[annotation['class_text']] = 1
                    else:
                        self.label_statistics[annotation['class_text']] = self.label_statistics[
                                                                         annotation['class_text']] + 1

            image = {'annotations': annotations, 'path': path, 'height': height, 'width': width}
            self.samples.append(image)
            self.number_samples += 1

        self.number_datasets += 1
        self.update_dataset_type(DatasetType.BOSCH)

        return images

    def read_all_lara_labels(self, input_txt, filter_labels=True):
        """
        Gets all labels from the LARA Traffic Light Dataset listed in the label file.

        Image Formats:
          - RGB 8bit, size = 640x480.

         LARA GT TXT File Format:
            Timestamp / frameindex x1 y1 x2 y2 id 'type' 'subtype'

            06:54.6118 / 5221 339 15 355 50 10 'Traffic Light' 'stop'
            06:54.6118 / 5221 342 131 348 146 35 'Traffic Light' 'ambiguous'
            06:54.6710 / 5222 338 13 354 48 10 'Traffic Light' 'stop'

        :param input_txt:      Path to GT TXT file.
        :param filter_labels:  If true only red, yellow, green and off labels are considered.

        :return: images: Labels for traffic lights. None in case the input file couldn't be found.
        """

        gt = pd.read_csv(input_txt, delimiter=' ', skiprows=13)
        tuples = [tuple(x) for x in gt.values]

        # FIXME: Add empty images to labels
        prev_frameindex = -1

        for i in range(len(tuples)):
            # convert to generic label format and generate some basic dataset statistics
            frameindex = tuples[i][2]
            path = os.path.abspath(os.path.join(os.path.dirname(input_txt), 'Lara3D_UrbanSeq1_JPG/frame_{:06d}.jpg'.format(int(frameindex))))

            # determine image size
            self.image_shape = self.get_image_shape(path)
            height = self.image_shape[0]
            width = self.image_shape[1]

            if frameindex > prev_frameindex:
                annotations = []

            # filter for relevant labels if activated
            if not filter_labels or \
                    tuples[i][10].lower().find('stop') >= 0 or \
                    tuples[i][10].lower().find('warning') >= 0 or \
                    tuples[i][10].lower().find('go') >= 0 or \
                    tuples[i][10].lower().find('ambiguous') >= 0:

                if tuples[i][10].lower().find('stop') >= 0:
                    label = 'red'
                elif tuples[i][10].lower().find('warning') >= 0:
                    label = 'yellow'
                elif tuples[i][10].lower().find('go') >= 0:
                    label = 'green'
                else:
                    label = 'off'

                class_id, class_text = self.get_class_id_text(label)
                annotation = {'class_id': class_id,
                              'class_text': class_text,
                              'x_max': tuples[i][5],
                              'x_min': tuples[i][3],
                              'y_max': tuples[i][6],
                              'y_min': tuples[i][4]}

                annotations.append(annotation)
                self.number_labeled_traffic_lights += 1

                if annotation['class_text'] not in self.label_statistics.keys():
                    self.label_statistics[annotation['class_text']] = 1
                else:
                    self.label_statistics[annotation['class_text']] = self.label_statistics[
                                                                     annotation['class_text']] + 1

            if frameindex > prev_frameindex:
                image = {'annotations': annotations, 'path': path, 'height': height, 'width': width}
                self.samples.append(image)
                self.number_samples += 1
                prev_frameindex = frameindex

        self.number_datasets += 1
        self.update_dataset_type(DatasetType.LARA)

        return self.samples

    def read_all_capstone_labels(self, input_yaml, filter_labels=True):
        """
        Gets all labels from the SDCND Capstone Traffic Light Dataset listed in the label file.

        Image Formats:
          - RGB 8bit, size = 1368x1096.

        SDCND Capstone Label File Format:

        - annotations:
          - {class: Green, x_width: 20.25524832517874, xmin: 646.3460457627134, y_height: 56.264578681052, ymin: 417.32241787431707}
          class: image
          filename: green/left0007.jpg

        :param input_yaml:     Path to yaml file.
        :param filter_labels:  If true only red, yellow, green and off labels are considered.

        :return: images: Labels for traffic lights. None in case the input file couldn't be found.
        """
        if not os.path.exists(input_yaml):
            return None

        images = yaml.load(open(input_yaml, 'rb').read())

        for i in range(len(images)):
            images[i]['filename'] = os.path.abspath(os.path.join(os.path.dirname(input_yaml), images[i]['filename']))

            # convert to generic label format and generate some basic dataset statistics
            annotations = []
            path = images[i]['filename']

            # determine image size
            self.image_shape = self.get_image_shape(path)
            height = self.image_shape[0]
            width = self.image_shape[1]

            for annotation in images[i]['annotations']:
                # filter for relevant labels if activated
                if not filter_labels or \
                        ((annotation['class'].lower().find('red') >= 0 and len(annotation['class']) == len('red')) or \
                         (annotation['class'].lower().find('yellow') >= 0 and len(annotation['class']) == len('yellow')) or \
                         (annotation['class'].lower().find('green') >= 0 and len(annotation['class']) == len('green')) or \
                         (annotation['class'].lower().find('off') >= 0 and len(annotation['class']) == len('off'))):

                    class_id, class_text = self.get_class_id_text(annotation['class'])
                    annotation = {'class_id': class_id,
                                  'class_text': class_text,
                                  'x_max': annotation['xmin'] + annotation['x_width'],
                                  'x_min': annotation['xmin'],
                                  'y_max': annotation['ymin'] + annotation['y_height'],
                                  'y_min': annotation['ymin']}

                    annotations.append(annotation)
                    self.number_labeled_traffic_lights += 1

                    if annotation['class_text'] not in self.label_statistics.keys():
                        self.label_statistics[annotation['class_text']] = 1
                    else:
                        self.label_statistics[annotation['class_text']] = self.label_statistics[
                                                                         annotation['class_text']] + 1

            image = {'annotations': annotations, 'path': path, 'height': height, 'width': width}
            self.samples.append(image)
            self.number_samples += 1

        self.number_datasets += 1
        self.update_dataset_type(DatasetType.SDCND_CAPSTONE)

        return images

    def read_predefined_dataset(self):
        """ Read all predefined datasets (Bosch, capstone real and capstone sim). """

        print('Loading Bosch dataset...', end='', flush=True)

        if self.read_all_bosch_labels(DATASET_BOSCH) is None:
            print('')
            print('ERROR: Input YAML file "{}" not found.'.format(DATASET_BOSCH))
            exit(-1)
        else:
            print('done')

        # print('Loading LARA dataset...', end='', flush=True)
        #
        # if self.read_all_lara_labels(DATASET_LARA) is None:
        #     print('')
        #     print('ERROR: Input TXT file "{}" not found.'.format(DATASET_LARA))
        #     exit(-1)
        # else:
        #     print('done')

        print('Loading SDCND Capstone real dataset...', end='', flush=True)

        if self.read_all_capstone_labels(DATASET_CAPSTONE_REAL) is None:
            print('')
            print('ERROR: Input TXT file "{}" not found.'.format(DATASET_CAPSTONE_REAL))
            exit(-1)
        else:
            print('done')

        print('Loading SDCND Capstone sim dataset...', end='', flush=True)

        if self.read_all_capstone_labels(DATASET_CAPSTONE_SIM) is None:
            print('')
            print('ERROR: Input TXT file "{}" not found.'.format(DATASET_CAPSTONE_SIM))
            exit(-1)
        else:
            print('done')

    def is_valid(self):
        """ Returns true if valid datasets resp. images are available. """
        return self.number_samples > 0

    def get_number_merged_datasets(self):
        """ Returns the number of merged datasets. """
        return self.number_datasets

    def split_dataset(self, train_ratio, shuffle=True):
        """ Shuffles and splits the dataset into a training and testing dataset.

        :param train_ratio: Size of the training dataset [%].
        :param shuffle:     If true, the dataset will be shuffeled before splitting.

        :return: Returns two lists, one with training and one with test samples.
        """
        train_samples, test_samples = train_test_split(self.samples, train_size=train_ratio, shuffle=shuffle)
        return train_samples, test_samples

    def generate_ground_truth_image(self, annotations, image_shape):
        """ Generates the ground truth image based on bounding boxes and the traffic light class.

        :param annotations: List with annotations.
        :param image_shape: Output image shape (width, height).

        :return: Ground truth image.
        """
        ground_truth = np.zeros((image_shape[0], image_shape[1], 3), dtype=np.uint8)

        for annotation in annotations:
            class_text = annotation['class_text']
            x_min = int(round(annotation['x_min']))
            x_max = int(round(annotation['x_max']))
            y_min = int(round(annotation['y_min']))
            y_max = int(round(annotation['y_max']))

            if class_text.find('red') >= 0:
                color = GT_TL_RED
            elif class_text.find('yellow') >= 0:
                color = GT_TL_YELLOW
            elif class_text.find('green') >= 0:
                color = GT_TL_GREEN
            else:
                color = GT_TL_UNDEFINED

            ground_truth[y_min:y_max, x_min:x_max, :] = color

        return ground_truth

    def generator(self, samples, batch_size=128, augmentation_rate=0.0, bbox=False, stop_at_end=False):
        """ Sample and ground truth generator.

        :param samples:           Samples which shall be loaded into memory.
        :param batch_size:        Batch size for actual run.
        :param augmentation_rate: Rate (0..1) of total images which will be randomly augmented.
                                  E.g. 0.6 augments 60% of the images and 40% are raw images
        :param bbox:              If true, the generator returns the sample with annotations (bounding boxes).
                                  Otherwise the ground truth image is returned.
        :param stop_at_end:       If true, the generator stops if all images have been provided.

        :return: Returns X (RGB image) and y (GT image).
        """
        number_samples = len(samples)
        number_generated_samples = 0
        running = True

        while running:  # loop forever so the generator never terminates
            for offset in range(0, self.number_samples, batch_size):
                batch_samples = samples[offset:offset + batch_size]

                number_batch_samples = 0
                images = []
                images_gt = []

                for batch_sample in batch_samples:
                    annotations = copy.deepcopy(batch_sample['annotations'])
                    image = cv2.imread(batch_sample['path'])
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    image_gt = self.generate_ground_truth_image(annotations, image.shape)

                    if augmentation_rate > 0.0 and np.random.rand() <= augmentation_rate:
                        # apply random translation
                        image, image_gt, annotations = da.DataAugmentation.random_translation(
                            image, image_gt, annotations, [70, 70], probability=0.5)

                        # apply random flip, lr_bias = 0.0 (no left/right bias correction of dataset)
                        image, image_gt, annotations = da.DataAugmentation.flip_image_horizontally(
                            image, image_gt, annotations, probability=0.5)

                        # apply random brightness
                        image = da.DataAugmentation.random_brightness(image, probability=0.5)

                    # final image pre-processing
                    if self.generator_image_shape[0] is not None:
                        image, image_gt, annotations = da.DataAugmentation.resize_image(
                            image, image_gt, annotations, self.generator_image_shape)
                        batch_sample['width'] = self.generator_image_shape[0]
                        batch_sample['height'] = self.generator_image_shape[1]

                    images.append(image)
                    images_gt.append(image_gt)
                    number_batch_samples += 1

                    if self.verbose:
                        # show generated images in a separate window
                        image = self.draw_bounding_boxes(image, annotations)
                        image_gen = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                        image_gt = cv2.cvtColor(image_gt, cv2.COLOR_RGB2BGR)
                        image_gen = cv2.addWeighted(image_gen, 1.0, image_gt, 0.3, 0.0)
                        image_gen = np.concatenate((image_gen, image_gt), axis=1)

                        self.plot_generator_heatmap(annotations)
                        cv2.imshow('Generator output (left: Generator Image + Ground Truth Overlay, right: Ground Truth)', image_gen)
                        cv2.waitKey(1)

                number_generated_samples += number_batch_samples

                if stop_at_end and number_generated_samples >= number_samples:
                    running = False

                if self.verbose:
                    print(' Generator: number_batch_samples: {:4d} number_generated_samples: {:5d}/{:5d}'.format(number_batch_samples, number_generated_samples, self.number_samples))

                # FIXME: Remove after debugging
                #if number_total_samples > self.number_samples:
                #    self.fig_heatmap_all.savefig('generator_label_heatmap_all.png')
                #    self.fig_heatmap.savefig('generator_label_heatmap_red_yellow_green_off.png')
                #    exit(0)

                # convert to numpy arrays
                X = np.array(images)
                if bbox:
                    y = np.array(batch_samples)
                else:
                    y = np.array(images_gt)
                yield shuffle(X, y)

    def plot_generator_heatmap(self, annotations):
        """ Plots a heatmap over all label positions (bounding boxes) generated by the generator.

        :param annotations:  List with annotations.
        """
        for annotation in annotations:
            class_text = annotation['class_text'].lower()
            x_max = int(round(annotation['x_max']))
            x_min = int(round(annotation['x_min']))
            y_max = int(round(annotation['y_max']))
            y_min = int(round(annotation['y_min']))

            if class_text.find('red') >= 0:
                self.generator_heatmap_red[y_min:y_max, x_min:x_max] += 1
            elif class_text.find('yellow') >= 0:
                self.generator_heatmap_yellow[y_min:y_max, x_min:x_max] += 1
            elif class_text.lower().find('green') >= 0:
                self.generator_heatmap_green[y_min:y_max, x_min:x_max] += 1
            elif class_text.find('off') >= 0:
                self.generator_heatmap_off[y_min:y_max, x_min:x_max] += 1

        self.generator_heatmap_all = self.generator_heatmap_red + self.generator_heatmap_yellow + \
                                     self.generator_heatmap_green + self.generator_heatmap_off

        # update label histogram
        self.plt_heatmap_all.set_data(self.generator_heatmap_all)
        self.plt_heatmap_all.autoscale()
        self.plt_heatmap_red.set_data(self.generator_heatmap_red)
        self.plt_heatmap_red.autoscale()
        self.plt_heatmap_yellow.set_data(self.generator_heatmap_yellow)
        self.plt_heatmap_yellow.autoscale()
        self.plt_heatmap_green.set_data(self.generator_heatmap_green)
        self.plt_heatmap_green.autoscale()
        self.plt_heatmap_off.set_data(self.generator_heatmap_off)
        self.plt_heatmap_off.autoscale()
        plt.draw_all()
        plt.pause(1e-9)

    def draw_bounding_boxes(self, image, annotations):
        """ Draw the bounding boxes

        :param image:       RGB Image
        :param annotations: List of annotations (bounding boxes)
        :return:
        """
        for annotation in annotations:
            color = (100, 100, 100)

            if annotation['class_text'].lower().find('red') >= 0:
                color = (255, 0, 0)
            elif annotation['class_text'].lower().find('yellow') >= 0:
                color = (255, 255, 0)
            elif annotation['class_text'].lower().find('green') >= 0:
                color = (0, 255, 0)

            x_max = int(round(annotation['x_max']))
            x_min = int(round(annotation['x_min']))
            y_max = int(round(annotation['y_max']))
            y_min = int(round(annotation['y_min']))

            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color)

        return image

    def print_statistics(self):
        """ Plots basic dataset statistics like number of images, labes, etc. """

        if self.number_samples == 0:
            print('ERROR: No valid dataset dictionary. Read datasets before.')
            return

        print()
        print(' Traffic Light Dataset')
        print('--------------------------------------------------')
        print('Number images:        {}'.format(self.number_samples))
        print('Number traffic light: {}'.format(self.number_labeled_traffic_lights))
        print('Image shape:          {}x{}x{}'.format(self.image_shape[1],
                                                      self.image_shape[0],
                                                      self.image_shape[2]))

    def plot_label_histogram(self, safe_figure=False):
        """ Plots a histogram over all labels.

        :param safe_figure:  If true safe figure as png file.
        """

        if self.number_samples == 0:
            print('ERROR: No valid dataset dictionary. Read datasets before.')
            return

        x = np.arange(len(self.label_statistics))
        label_names = np.array([], dtype=np.str)
        label_hist = np.array([])
        colors = []

        print()
        print(' Label Class Distribution')
        print('--------------------------------------------------')

        for key in sorted(self.label_statistics.keys()):
            print('{:18s} = {}'.format(key, self.label_statistics[key]))
            label_names = np.append(label_names, key)
            label_hist = np.append(label_hist, self.label_statistics[key])

            # set bar color depending on label class
            if str(key).lower().find('red') >= 0:
                colors.append(plu.COLOR_RED)
            elif str(key).lower().find('yellow') >= 0:
                colors.append(plu.COLOR_YELLOW)
            elif str(key).lower().find('green') >= 0:
                colors.append(plu.COLOR_GREEN)
            else:
                colors.append(plu.COLOR_GREY)

        # plot label histogram
        if self.dataset_type == DatasetType.BOSCH:
            title = 'Label Distribution in Bosch Small Traffic Light Dataset'
        elif self.dataset_type == DatasetType.LARA:
            title = 'Label Distribution in LARA Traffic Light Dataset'
        elif self.dataset_type == DatasetType.SDCND_CAPSTONE:
            title = 'Label Distribution in SDCND Capstone Traffic Light Dataset'
        else:
            title = 'Label Distribution in Merged Traffic Light Dataset'

        fig, ax = plt.subplots()
        fig.subplots_adjust(left=0.13, right=0.97, bottom=0.1, top=0.9)
        rect = plt.barh(x, label_hist, color=colors)
        plu.autolabel_barh(rect, ax)
        plt.yticks(x, label_names)
        plt.title(title)
        plt.ylabel('label class')
        plt.xlabel('number of labels per class')
        ax.set_axisbelow(True)
        # ax.minorticks_on()
        ax.grid(which='major', linestyle='-', linewidth='0.5', color='grey')
        # ax.grid(which='minor', linestyle=':', linewidth='0.5', color='grey')

        if safe_figure:
            plt.savefig('label_histogram.png')

        plt.show(block=False)

    def plot_label_heatmap(self, safe_figure=False):
        """ Plots a heatmap over all label positions (bounding boxes).

        :param safe_figure: If true safe figure as png file.
        """

        if self.number_samples == 0:
            print('ERROR: No valid dataset dictionary. Read datasets before.')
            return

        heatmap_red = np.zeros((self.image_shape[0], self.image_shape[1]), dtype=np.float)
        heatmap_yellow = np.zeros((self.image_shape[0], self.image_shape[1]), dtype=np.float)
        heatmap_green = np.zeros((self.image_shape[0], self.image_shape[1]), dtype=np.float)
        heatmap_off = np.zeros((self.image_shape[0], self.image_shape[1]), dtype=np.float)

        for i in range(self.number_samples):
            for annotation in self.samples[i]['annotations']:
                class_text = annotation['class_text'].lower()
                x_max = int(round(annotation['x_max']))
                x_min = int(round(annotation['x_min']))
                y_max = int(round(annotation['y_max']))
                y_min = int(round(annotation['y_min']))

                if class_text.find('red') >= 0:
                    heatmap_red[y_min:y_max, x_min:x_max] += 1
                elif class_text.find('yellow') >= 0:
                    heatmap_yellow[y_min:y_max, x_min:x_max] += 1
                elif class_text.lower().find('green') >= 0:
                    heatmap_green[y_min:y_max, x_min:x_max] += 1
                elif class_text.find('off') >= 0:
                    heatmap_off[y_min:y_max, x_min:x_max] += 1

        heatmap_all = heatmap_red + heatmap_yellow + heatmap_green + heatmap_off

        # plot label histogram
        if self.dataset_type == DatasetType.BOSCH:
            title = 'Label Heatmap of Bosch Small Traffic Light Dataset'
        elif self.dataset_type == DatasetType.LARA:
            title = 'Label Heatmap of LARA Traffic Light Dataset'
        elif self.dataset_type == DatasetType.SDCND_CAPSTONE:
            title = 'Label Heatmap of SDCND Capstone Traffic Light Dataset'
        else:
            title = 'Label Heatmap of Merged Traffic Light Dataset'

        fig, ax = plt.subplots()
        fig.subplots_adjust(left=0.1, right=0.97, bottom=0.1, top=0.9)
        fig.suptitle(title)
        ax.imshow(heatmap_all, cmap='jet', interpolation='nearest')
        ax.set_title('Red, Green, Red, Off TL')

        if safe_figure:
            plt.savefig('label_heatmap_all.png')

        fig, axarr = plt.subplots(2, 2)
        fig.subplots_adjust(left=0.1, right=0.97, bottom=0.1, top=0.9)
        fig.suptitle(title)

        axarr[0, 0].imshow(heatmap_red, cmap='Reds', interpolation='nearest')
        axarr[0, 0].set_title('Red TL')
        axarr[0, 1].imshow(heatmap_yellow, cmap='Oranges', interpolation='nearest')
        axarr[0, 1].set_title('Yellow TL')
        axarr[1, 0].imshow(heatmap_green, cmap='Greens', interpolation='nearest')
        axarr[1, 0].set_title('Green TL')
        axarr[1, 1].imshow(heatmap_off, cmap='Greys', interpolation='nearest')
        axarr[1, 1].set_title('Off TL')

        if safe_figure:
            plt.savefig('label_heatmap_red_yellow_green_off.png')

        plt.show(block=False)

    def show_labeled_images(self, output_folder=None):
        """
        Shows all images with colored labeled traffic lights.

        :param output_folder: If None, do not save picture. Else enter path to folder.
        """

        if output_folder is not None:
            if not os.path.exists(output_folder):
                os.makedirs(output_folder)

        for i, label in enumerate(self.samples):
            image = cv2.imread(label['path'])

            if image is None:
                raise IOError('Could not open image path', label['path'])

            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = self.draw_bounding_boxes(image, label['annotations'])
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imshow('Labeled Images', image)
            cv2.waitKey(10)

            if output_folder is not None:
                cv2.imwrite(os.path.join(output_folder, str(i).zfill(10) + '_' + os.path.basename(label['path'])), image)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dataset handler provides statistics and label transformation.')

    parser.add_argument(
        '--bosch_label_file',
        help='Path to the Bosch label YAML file.',
        dest='bosch_label_file',
        metavar='YAML_file'
    )

    parser.add_argument(
        '--lara_label_file',
        help='Path to the LARA label XML file.',
        dest='lara_label_file',
        metavar='XML_file'
    )

    parser.add_argument(
        '--capstone_label_file',
        help='Path to the SDCND Capstone label YAML file.',
        dest='capstone_label_file',
        metavar='YAML_file'
    )

    parser.add_argument(
        '-df', '--disable_filter',
        help='If set, all labels will be added to the merged dataset. Otherwise red, yellow, green, off only.',
        action='store_true',
        default=False,
        dest='disable_filter'
    )

    parser.add_argument(
        '-s', '--statistics',
        help='Show dataset statistics like label distribution.',
        action='store_true',
        default=False,
        dest='statistics'
    )

    parser.add_argument(
        '-sp', '--safe_plots',
        help='Safe plots as PNG files.',
        action='store_true',
        default=False,
        dest='safe_plots'
    )

    parser.add_argument(
        '-si', '--show_images',
        help='Show all labeled images in the dataset in a video.',
        action='store_true',
        default=False,
        dest='show_images'
    )

    parser.add_argument(
        '-sg', '--show_generator',
        help='Show generator images and labels in a video.',
        action='store_true',
        default=False,
        dest='show_generator'
    )

    args = parser.parse_args()

    if len(sys.argv) < 4:
        # no arguments found
        parser.print_usage()
        parser.exit(-1)

    safe_plots = False

    dataset_handler = DatasetHandler(width=640, height=480, verbose=args.show_generator)

    if args.bosch_label_file:
        print('Loading Bosch dataset...', end='', flush=True)

        if dataset_handler.read_all_bosch_labels(args.bosch_label_file, filter_labels=not args.disable_filter) is None:
            print('')
            print('ERROR: Input YAML file "{}" not found.'.format(args.bosch_label_file))
            exit(-1)
        else:
            print('done')

    if args.lara_label_file:
        print('Loading LARA dataset...', end='', flush=True)

        if dataset_handler.read_all_lara_labels(args.lara_label_file, filter_labels=not args.disable_filter) is None:
            print('')
            print('ERROR: Input XML file "{}" not found.'.format(args.lara_label_file))
            exit(-1)
        else:
            print('done')

    if args.capstone_label_file:
        print('Loading SDCND Capstone dataset...', end='', flush=True)

        if dataset_handler.read_all_capstone_labels(args.capstone_label_file, filter_labels=not args.disable_filter) is None:
            print('')
            print('ERROR: Input YAML file "{}" not found.'.format(args.capstone_label_file))
            exit(-1)
        else:
            print('done')

    if dataset_handler.is_valid():
        if args.statistics:
            dataset_handler.print_statistics()
            dataset_handler.plot_label_histogram(safe_figure=args.safe_plots)
            dataset_handler.plot_label_heatmap(safe_figure=args.safe_plots)
            plt.show()
        elif args.show_images:
            print('Exit with CTRL+C')
            dataset_handler.show_labeled_images(output_folder=None)
        elif args.show_generator:
            generator = dataset_handler.generator(dataset_handler.samples, batch_size=1, augmentation_rate=0.65)

            for x_train, y_train in generator:
                pass
    else:
        print('ERROR: No valid datasets found.')
        exit(-1)
