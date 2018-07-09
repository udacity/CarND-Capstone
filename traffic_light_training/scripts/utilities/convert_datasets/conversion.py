# This script is for converting the data from the LISA Traffic Light Dataset
# https://www.kaggle.com/mbornoe/lisa-traffic-light-dataset/home

# This script uses code from
# https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/using_your_own_dataset.md

import os
import glob
import csv
import yaml
import cv2
import tqdm
import hashlib
import random
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt

# Name of the Annotation File
annotation_file = 'frameAnnotationsBOX.csv'


def write_tf_record(tf_record_filename, image_boxes_list: list):
    # Create TF Record writer
    rec_writer = tf.python_io.TFRecordWriter(tf_record_filename)

    # Dict 'image_boxes_list' is filled and has to be shuffled now
    for i in range(10):
        random.shuffle(image_boxes_list)

    print('Reading images.. converting to TFRecords..')
    print('')
    for image_element in tqdm.tqdm(image_boxes_list):
        # Not done here: Converting to proper RGB (since OpenCV uses BGR)
        img_path = list(image_element.keys())[0]
        img = cv2.imread(img_path)

        # Create TFRecord
        tf_record = convert_to_tfrecord(img,
                                        img_path,
                                        image_element[img_path]['boxes'])

        rec_writer.write(tf_record.SerializeToString())

    # Close the writer
    rec_writer.close()


def read_lisa_data(base_path_images, base_path_annotations):

    # Find all annotation files to read in
    annotation_files = glob.glob(os.path.join(base_path_annotations, '*', annotation_file), recursive=True)

    # Read image paths and boxes in memory
    image_boxes = {}

    print('Reading annotation files..')
    print('')

    # Iterate the files
    for file in tqdm.tqdm(annotation_files):

        # Create CSV Reader
        reader = csv.reader(open(file, 'r'), delimiter=';')
        next(reader, None)
        for csv_row in reader:
            # A little bit hacky to parse the filepath and names
            image_file_name = os.path.join(base_path_images,
                                           os.path.split(csv_row[6])[0], 'frames',
                                           os.path.basename(csv_row[0]))

            # Check if this image is already loaded or not
            if image_file_name not in image_boxes:
                # Put to dictionary
                image_boxes[image_file_name] = {'boxes': []}

            # "stop" will be mapped to 0 (red), "go" will be mapped to 2 (green)
            # "warning" will be mapped to 1 (yellow)
            label = None
            label_text = csv_row[1]
            if 'stop' in label_text:
                label = 0
            elif 'warning' in label_text:
                label = 1
            elif 'go' in label_text:
                label = 2

            if label is None:
                raise ValueError('No valid label found')

            x = int(csv_row[2])
            y = int(csv_row[3])
            width = int(csv_row[4]) - x
            height = int(csv_row[5]) - y

            # Push to dictionary
            image_boxes[image_file_name]['boxes'].append({
                'classname': label_text,
                'classid': label,
                'x': x,
                'y': y,
                'width': width,
                'height': height
            })

            # For Debugging, display the data
            debug = False
            if debug:
                print(label, x, y, width, height)
                # plt.imshow(img[y:y + height, x:x + width, :])
                plt.show()

    listed_images = []
    for k in image_boxes:
        listed_images.append({k: image_boxes[k]})

    return listed_images


def read_bosch_data(train_yaml_bosch, base_path_images, use_basename=False):

    if not os.path.isfile(train_yaml_bosch):
        raise FileNotFoundError(train_yaml_bosch + ' not found')

    # Read the yaml document
    document = yaml.load(open(train_yaml_bosch, 'r'))

    # Read into 'own' structure
    image_boxes = {}

    print('Reading annotation files..')
    print('')
    for description in tqdm.tqdm(document):
        # Only take images with traffic lights in it
        if len(description['boxes']) > 0:
            name = description['path'][2:] if not use_basename else os.path.basename(description['path'])
            image_path = os.path.join(base_path_images, name)
            image_boxes[image_path] = {'boxes': []}

            for box in description['boxes']:

                xmin = min(box['x_min'], box['x_max'])
                xmax = max(box['x_min'], box['x_max'])
                ymin = min(box['y_min'], box['y_max'])
                ymax = max(box['y_min'], box['y_max'])

                x = xmin
                y = ymin
                width = xmax - x
                height = ymax - y
                label_text = box['label']

                if width < 0 or height < 0:
                    raise ValueError('Invalid width or height.')

                label = None
                if 'Red' in label_text:
                    label = 0
                elif 'Yellow' in label_text:
                    label = 1
                elif 'Green' in label_text:
                    label = 2
                elif 'off' in label_text:
                    label = 3

                if label is None:
                    raise ValueError('No valid label found')

                image_boxes[image_path]['boxes'].append({
                    'classname': label_text,
                    'classid': label,
                    'x': x,
                    'y': y,
                    'width': width,
                    'height': height
                })

    listed_images = []
    for k in image_boxes:
        listed_images.append({k: image_boxes[k]})

    return listed_images


def read_tf_record(filename):
    if os.path.isfile(filename) is False:
        raise FileNotFoundError('Could not find ' + filename)

    record_iterator = tf.python_io.tf_record_iterator(filename)
    tmp_folder = 'tmp'
    if os.path.isdir(tmp_folder) is False:
        os.makedirs(tmp_folder)

    image_boxes = {}
    for string_record in record_iterator:
        example = tf.train.Example()
        example.ParseFromString(string_record)

        img_fn = example.features.feature['image/source_id'].bytes_list.value[0].decode('utf-8')
        height = int(example.features.feature['image/height'].int64_list.value[0])
        width = int(example.features.feature['image/width'].int64_list.value[0])

        img_string = (example.features.feature['image/encoded'].bytes_list.value[0])

        bbox_xmin = np.array(example.features.feature['image/object/bbox/xmin'].float_list.value) * width
        bbox_xmax = np.array(example.features.feature['image/object/bbox/xmax'].float_list.value) * width
        bbox_ymin = np.array(example.features.feature['image/object/bbox/ymin'].float_list.value) * height
        bbox_ymax = np.array(example.features.feature['image/object/bbox/ymax'].float_list.value) * height

        class_text = np.array(example.features.feature['image/object/class/text'].bytes_list.value)
        # class_label = np.array(example.features.feature['image/object/class/label'].int64_list.value)

        img_1d = np.fromstring(img_string, dtype=np.uint8)
        img = cv2.imdecode(img_1d, -1)

        img_path = os.path.join(tmp_folder, img_fn)
        if os.path.isfile(img_path) is False:
            cv2.imwrite(img_path, img)

        # Check if this image is already loaded or not
        if img_fn not in image_boxes:
            # Put to dictionary
            image_boxes[img_path] = {'boxes': []}

        for i in range(class_text.shape[0]):
            label = None
            class_name = class_text[i].decode('utf-8')
            if 'red' in class_name:
                label = 0
            elif 'yellow' in class_name:
                label = 1
            elif 'green' in class_name:
                label = 2
            elif 'off' in class_name:
                label = 3
            elif 'none' in class_name:
                label = 3

            if label is None:
                raise ValueError('No valid label found')

            image_boxes[img_path]['boxes'].append({
                'classname': class_name,
                'classid': label,
                'x': int(round(bbox_xmin[i])),
                'y': int(round(bbox_ymin[i])),
                'width': int(round(bbox_xmax[i] - bbox_xmin[i])),
                'height': int(round(bbox_ymax[i] - bbox_ymin[i]))
            })

    listed_images = []
    for k in image_boxes:
        listed_images.append({k:image_boxes[k]})

    return listed_images


def convert_to_tfrecord(image, filename, list_of_bb_dicts):
    """ Parts are from
    https://github.com/tensorflow/models/blob/master/research/object_detection/dataset_tools/create_kitti_tf_record.py
    """

    # First encode to jpg
    format = b'jpg'
    encoded_image = cv2.imencode('.jpg', image)[1].tostring()

    key = hashlib.sha256(encoded_image).hexdigest()

    img_width = int(image.shape[1])
    img_height = int(image.shape[0])

    class_names = []
    class_ids = []
    xmin_norm = []
    ymin_norm = []
    xmax_norm = []
    ymax_norm = []

    for box in list_of_bb_dicts:

        bb_x = box['x']
        bb_y = box['y']
        bb_width = box['width']
        bb_height = box['height']

        class_names.append(box['classname'].encode('utf8'))
        class_ids.append(box['classid'] + 1)  # +1 since the google object detection api works with 0 as background

        # Normalize bounding box positions
        xmin_norm.append(bb_x / float(img_width))
        ymin_norm.append(bb_y / float(img_height))
        xmax_norm.append((bb_x + bb_width) / float(img_width))
        ymax_norm.append((bb_y + bb_height) / float(img_height))

        if xmin_norm[-1] > xmax_norm[-1] or ymin_norm[-1] > ymax_norm[-1]:
            raise ValueError('Invalid boxes found for ' + filename + ' !')

    tf_record = tf.train.Example(features=tf.train.Features(feature={
        'image/height': tf.train.Feature(int64_list=tf.train.Int64List(value=[img_height])),
        'image/width': tf.train.Feature(int64_list=tf.train.Int64List(value=[img_width])),
        'image/filename': tf.train.Feature(bytes_list=tf.train.BytesList(value=[filename.encode('utf8')])),
        'image/source_id': tf.train.Feature(bytes_list=tf.train.BytesList(value=[filename.encode('utf8')])),
        'image/key/sha256': tf.train.Feature(bytes_list=tf.train.BytesList(value=[key.encode('utf8')])),
        'image/encoded': tf.train.Feature(bytes_list=tf.train.BytesList(value=[encoded_image])),
        'image/format': tf.train.Feature(bytes_list=tf.train.BytesList(value=[format])),
        'image/object/bbox/xmin': tf.train.Feature(float_list=tf.train.FloatList(value=xmin_norm)),
        'image/object/bbox/xmax': tf.train.Feature(float_list=tf.train.FloatList(value=xmax_norm)),
        'image/object/bbox/ymin': tf.train.Feature(float_list=tf.train.FloatList(value=ymin_norm)),
        'image/object/bbox/ymax': tf.train.Feature(float_list=tf.train.FloatList(value=ymax_norm)),
        'image/object/class/text': tf.train.Feature(bytes_list=tf.train.BytesList(value=class_names)),
        'image/object/class/label': tf.train.Feature(int64_list=tf.train.Int64List(value=class_ids)),
    }))

    return tf_record


def merge_datasets(trains: list, validations: list, equalize: bool):
    merged_trains = []
    merged_vals = []

    if not equalize:
        for train in trains:
            merged_trains.extend(train)
        for val in validations:
            merged_vals.extend(val)
    else:
        # Equalize the frequency of the domain-specific images
        for i, datasets in enumerate([trains, validations]):
            max_occurences = 0
            for dataset in datasets:
                max_occurences = max(len(dataset), max_occurences)
            # Fill up every dataset with random sampled images
            # So that the number of images in the dataset equals to the biggest dataset.
            for k, dataset in enumerate(datasets):
                occurences = len(dataset)
                buffer_dataset = []
                for j in range(max_occurences - occurences):
                    buffer_dataset.append(random.choice(dataset))

                datasets[k].extend(buffer_dataset)

            for dataset in datasets:
                if i == 0:
                    merged_trains.extend(dataset)
                else:
                    merged_vals.extend(dataset)

    return merged_trains, merged_vals


if __name__ == "__main__":
    # Set the base path to Lisa Dataset Training directories
    # Since we only train on day, this points to dayTrain
    train_base_path_annotations = '/data/Datasets/LisaTrafficLight/Annotations/dayTrain/'
    train_base_path_images = '/data/Datasets/LisaTrafficLight'

    val_base_path_annotations = '/data/Datasets/LisaTrafficLight/Annotations/dayVal/'
    val_base_path_images = '/data/Datasets/LisaTrafficLight'

    lisa_dataset_train = read_lisa_data(train_base_path_images, train_base_path_annotations)
    lisa_dataset_val = read_lisa_data(val_base_path_images, val_base_path_annotations)

    # Read in all the Bosch Dataset Meta data
    base_path_images_bosch = '/data/Datasets/Bosch Traffic Lights Dataset'
    base_path_images_val_bosch = '/data/Datasets/Bosch Traffic Lights Dataset/rgb/test/'

    train_yaml_bosch = '/data/Datasets/Bosch Traffic Lights Dataset/train.yaml'
    val_yaml_bosch = '/data/Datasets/Bosch Traffic Lights Dataset/test.yaml'

    bosch_dataset_train = read_bosch_data(train_yaml_bosch, base_path_images_bosch)
    bosch_dataset_val = read_bosch_data(val_yaml_bosch, base_path_images_val_bosch, use_basename=True)

    # Read TFRecords for simulator data and course data
    train_record_site = '/data/Datasets/UdacityData/traffic-light-site-train.record'
    train_record_sim = '/data/Datasets/UdacityData/traffic-light-sim-train.record'

    val_record_site = '/data/Datasets/UdacityData/traffic-light-site-test.record'
    val_record_sim = '/data/Datasets/UdacityData/traffic-light-sim-test.record'

    site_dataset_train = read_tf_record(train_record_site)
    site_dataset_val = read_tf_record(train_record_site)

    sim_dataset_train = read_tf_record(train_record_sim)
    sim_dataset_val = read_tf_record(val_record_sim)

    # Merge data
    trains, vals = merge_datasets(
                    [
                        lisa_dataset_train,
                        bosch_dataset_train,
                        site_dataset_train,
                        sim_dataset_train
                    ],
                    [
                        lisa_dataset_val,
                        bosch_dataset_val,
                        site_dataset_val,
                        sim_dataset_val
                    ],
                    equalize=True)

    # Convert all to TFRecord
    train_TFRecord_name = 'train_records.tfrecord'
    val_TFRecord_name = 'val_records.tfrecord'

    write_tf_record(train_TFRecord_name, trains)
    write_tf_record(val_TFRecord_name, vals)








