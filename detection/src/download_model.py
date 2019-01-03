import os
import six.moves.urllib as urllib
import tarfile
import tensorflow as tf

flags = tf.app.flags
flags.DEFINE_string('model_name', 'faster_rcnn_resnet101_coco_11_06_2017', 'Model name')
flags.DEFINE_string('output_path', 'data', 'Output path')
flags.DEFINE_string('download_base', 'http://download.tensorflow.org/models/object_detection/', 'Download base URL')
FLAGS = flags.FLAGS


def download(url, output_path, file_name):
    print("Downloading: {} to {}/{}".format(url, output_path, file_name))
    opener = urllib.request.URLopener()
    opener.retrieve(url, os.path.join(output_path, file_name))
    tar_file = tarfile.open(os.path.join(output_path, file_name))
    tar_file.extractall(output_path)

def main(args):
    # # What model to download.
    # MODEL_NAME = 'faster_rcnn_resnet101_coco_11_06_2017'
    # MODEL_FILE = MODEL_NAME + '.tar.gz'
    # DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'
    # WORK_DIR = '../models'
    # writer = tf.python_io.TFRecordWriter(FLAGS.output_file)
    # labels = label_map_util.get_label_map_dict(FLAGS.label_map)
    # examples = yaml.load(open(FLAGS.input_file, 'rb').read())

    download(FLAGS.download_base + FLAGS.model_name + ".tar.gz", FLAGS.output_path, FLAGS.model_name + ".tar.gz")


if __name__ == '__main__':
    tf.app.run()