import tensorflow as tf
import yaml
import os
from object_detection.utils import dataset_util, label_map_util

flags = tf.app.flags
flags.DEFINE_string('output_file', '', 'File to output TFRecord')
flags.DEFINE_string('input_file', '', 'YAML input file')
flags.DEFINE_string('label_map', '', 'Label input file')
flags.DEFINE_integer('height', 600, 'Image height')
flags.DEFINE_integer('width', 800, 'Image width')
FLAGS = flags.FLAGS

def create_tf_example(example, labelmap):
    # Udacity sim data set
    height = FLAGS.height
    width = FLAGS.width

    filename = example['filename']  # Filename of the image. Empty if image is not from file
    filename = filename.encode()

    with tf.gfile.GFile(example['filename'], 'rb') as fid:
        encoded_image = fid.read()

    image_format = 'jpg'.encode()

    xmins = []  # List of normalized left x coordinates in bounding box (1 per box)
    xmaxs = []  # List of normalized right x coordinates in bounding box
    # (1 per box)
    ymins = []  # List of normalized top y coordinates in bounding box (1 per box)
    ymaxs = []  # List of normalized bottom y coordinates in bounding box
    # (1 per box)
    classes_text = []  # List of string class name of bounding box (1 per box)
    classes = []  # List of integer class id of bounding box (1 per box)

    for box in example['annotations']:
        # if box['occluded'] is False:
        # print("adding box")
        xmins.append(float(box['xmin'] / width))
        xmaxs.append(float((box['xmin'] + box['x_width']) / width))
        ymins.append(float(box['ymin'] / height))
        ymaxs.append(float((box['ymin'] + box['y_height']) / height))
        classes_text.append(box['class'].encode())
        classes.append(int(labelmap[box['class']]))

    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(filename),
        'image/source_id': dataset_util.bytes_feature(filename),
        'image/encoded': dataset_util.bytes_feature(encoded_image),
        'image/format': dataset_util.bytes_feature(image_format),
        'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
        'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
        'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
        'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
        'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
        'image/object/class/label': dataset_util.int64_list_feature(classes),
    }))

    return tf_example


def main(args):
    writer = tf.python_io.TFRecordWriter(FLAGS.output_file)
    labels = label_map_util.get_label_map_dict(FLAGS.label_map)
    examples = yaml.load(open(FLAGS.input_file, 'rb').read())

    for example in examples:
        example['filename'] = os.path.abspath(
            os.path.join(os.path.dirname(FLAGS.input_file), example['filename']))
        tf_example = create_tf_example(example, labels)
        writer.write(tf_example.SerializeToString())

    writer.close()


if __name__ == '__main__':
    tf.app.run()