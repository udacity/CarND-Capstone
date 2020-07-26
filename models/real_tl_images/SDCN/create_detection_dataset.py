"""Data conversion from Udacity traffic light dataset to TFRecord.

Example usage:
    python create_udacity_sim_tf_record.py  \
        --input_yaml=/home/user/annotation.yaml \
        --train_output_path=/home/user/train_data.record
        --valid_output-path=/home/user/valid_data.record
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import hashlib
import io
import yaml
import os

import PIL.Image
import tensorflow as tf

from sklearn.model_selection import train_test_split
from object_detection.utils import dataset_util


flags = tf.app.flags
flags.DEFINE_string('train_output_path', '', 'Path to output TFRecord')
flags.DEFINE_string('valid_output_path', '', 'Path to output TFRecord')
flags.DEFINE_string('input_yaml', '', 'Path to annotation yaml file')

FLAGS = flags.FLAGS

# LABELS_MAP = {
#     "Green" : 1,
#     "Red" : 2,
#     "Yellow" : 3,
#     "Unknown" : 4
# }
LABELS_MAP = {
    "Green" : 1,
    "Red" : 1,
    "Yellow" : 1,
    "Unknown" : 2,
}

def dict_to_tf_example(example):

    filename = example['filename']
    filename = filename.encode()

    with tf.gfile.GFile(example['filename'], 'rb') as fid:
        encoded_jpg = fid.read()

    encoded_jpg_io = io.BytesIO(encoded_jpg)
    image = PIL.Image.open(encoded_jpg_io)

    width , height = image.size

    if image.format != 'JPEG':
        raise ValueError('Image format not JPEG')
    key = hashlib.sha256(encoded_jpg).hexdigest()


    xmins = []         # left x-coordinate
    ymins = []         # right x-coordinate
    xmaxs = []         # top y-coordinate
    ymaxs = []         # buttom y-coordinate
    classes = []       # class id
    classes_text = []  # class name

    for box in example['annotations']:
        xmins.append(float(box['xmin'] / width))
        xmaxs.append(float((box['xmin'] + box['x_width']) / width))
        ymins.append(float(box['ymin'] / height))
        ymaxs.append(float((box['ymin'] + box['y_height']) / height))
        classes_text.append(box['class'].encode())
        classes.append(int(LABELS_MAP[box['class']]))



    tf_example = tf.train.Example(features=tf.train.Features(feature={
      'image/height': dataset_util.int64_feature(height),
      'image/width': dataset_util.int64_feature(width),
      'image/filename': dataset_util.bytes_feature(filename),
      'image/source_id': dataset_util.bytes_feature(filename),
      'image/key/sha256': dataset_util.bytes_feature(key.encode('utf8')),
      'image/encoded': dataset_util.bytes_feature(encoded_jpg),
      'image/format': dataset_util.bytes_feature('jpeg'.encode('utf8')),
      'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
      'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
      'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
      'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
      'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
      'image/object/class/label': dataset_util.int64_list_feature(classes)
    }))

    return tf_example


def main(_):

  examples = yaml.load(open(FLAGS.input_yaml, 'rb').read())

  for i in range(len(examples)):
      examples[i]['filename'] = os.path.abspath(os.path.join(os.path.dirname(FLAGS.input_yaml), examples[i]['filename']))

  example_train, example_valid = train_test_split(examples, test_size=0.2, random_state=42)

  with tf.python_io.TFRecordWriter(FLAGS.train_output_path) as writer:
    for ex in example_train:
        tf_example = dict_to_tf_example(ex)
        writer.write(tf_example.SerializeToString())

  with tf.python_io.TFRecordWriter(FLAGS.valid_output_path) as writer:
    for ex in example_valid:
        tf_example = dict_to_tf_example(ex)
        writer.write(tf_example.SerializeToString())

if __name__ == '__main__':
    tf.app.run()
