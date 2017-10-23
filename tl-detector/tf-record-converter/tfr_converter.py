import tensorflow as tf
from object_detection.utils import dataset_util
import xml.etree.ElementTree as et
import numpy as np
import os


class Sample:
    def __init__(self):
        self.height = 0
        self.width = 0
        self.filename = ''
        self.image_format = b'jpg'
        self.dir = ''

        self.xmins = []
        self.xmaxs = []
        self.ymins = []
        self.ymaxs = []
        self.classes_text = []
        self.classes = []


def convert_label_to_id(label):
    # TODO: Convert this hard-coded labels to use a label map.
    if label == 'red':
        return 1
    elif label == 'yellow':
        return 2
    elif label == 'green':
        return 3
    else:
        return 4


def convert_xml_to_sample(xml_path, subdir):
    tree = et.parse(xml_path)
    root = tree.getroot()
    sample = Sample()

    sample.width = int(root.find('size')[0].text)
    sample.height = int(root.find('size')[1].text)
    sample.filename = root.find('filename').text.encode('utf8')
    sample.dir = subdir.encode('utf8')

    for member in root.findall('object'):
        label = member[0].text.encode('utf8')
        label_id = convert_label_to_id(label)
        xmin = float(member[4][0].text)/float(sample.width)
        ymin = int(member[4][1].text)/float(sample.height)
        xmax = int(member[4][2].text)/float(sample.width)
        ymax = int(member[4][3].text)/float(sample.height)

        sample.classes_text.append(label)
        sample.classes.append(label_id)
        sample.xmins.append(xmin)
        sample.xmaxs.append(xmax)
        sample.ymins.append(ymin)
        sample.ymaxs.append(ymax)

    return sample


def split_samples(samples, rate=0.2):
    np.random.shuffle(samples)
    split_index = int(len(samples) * rate)
    test_set, train_set = samples[:split_index], samples[split_index:]
    return test_set, train_set


def create_samples(path):
    samples = []
    for subdir, dirs, files in os.walk(path):
        for f in files:
            if "xml" in f:
                xml_path = os.path.join(subdir, f)
                sample = convert_xml_to_sample(xml_path, subdir)
                samples.append(sample)
    return samples


def create_tf_example(sample):

    with tf.gfile.GFile(os.path.join(sample.dir, sample.filename), 'rb') as fid:
        encoded_image_data = fid.read()
    height = sample.height
    width = sample.width
    filename = sample.filename.encode('utf8')
    image_format = b'jpg'

    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(filename),
        'image/source_id': dataset_util.bytes_feature(filename),
        'image/encoded': dataset_util.bytes_feature(encoded_image_data),
        'image/format': dataset_util.bytes_feature(image_format),
        'image/object/bbox/xmin': dataset_util.float_list_feature(sample.xmins),
        'image/object/bbox/xmax': dataset_util.float_list_feature(sample.xmaxs),
        'image/object/bbox/ymin': dataset_util.float_list_feature(sample.ymins),
        'image/object/bbox/ymax': dataset_util.float_list_feature(sample.ymaxs),
        'image/object/class/text': dataset_util.bytes_list_feature(sample.classes_text),
        'image/object/class/label': dataset_util.int64_list_feature(sample.classes),
    }))
    return tf_example


def create_record(samples, output_path):
    writer = tf.python_io.TFRecordWriter(output_path)
    for sample in samples:
        tf_example = create_tf_example(sample)
        writer.write(tf_example.SerializeToString())
    writer.close()


def main(_):
    # TODO: Create a way for this script to use shell arguments to specify the input/output.
    print("Creating TF Records...")
    samples = create_samples("./")
    test_set, train_set = split_samples(samples)
    create_record(test_set, "test.record")
    create_record(train_set, "train.record")
    print("Records Created!")


if __name__ == '__main__':
    tf.app.run()
