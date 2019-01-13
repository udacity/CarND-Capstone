import os
import six.moves.urllib as urllib
import tarfile
import tensorflow as tf
import hashlib

flags = tf.app.flags
flags.DEFINE_string('model_name', 'faster_rcnn_resnet101_coco_11_06_2017', 'Model name')
flags.DEFINE_string('output_path', 'out', 'Output path')
flags.DEFINE_string('download_path', '/tmp', 'Output path')
flags.DEFINE_string('download_base', 'http://download.tensorflow.org/models/object_detection/', 'Download base URL')
FLAGS = flags.FLAGS

def md5(fname):
    hash_md5 = hashlib.md5()
    with open(fname, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()


def is_downloaded(file, md5sum):
    return os.path.isfile(file) and md5(file) == md5sum


def download(url, download_path, output_path, file_name):
    print("Downloading: {} to {}/{}".format(url, download_path, file_name))
    opener = urllib.request.URLopener()
    opener.retrieve(url, os.path.join(download_path, file_name))
    tar_file = tarfile.open(os.path.join(download_path, file_name))
    tar_file.extractall(output_path)


def main(args):
    url = FLAGS.download_base + FLAGS.model_name + ".tar.gz"
    file_name = FLAGS.model_name + ".tar.gz"
    if not is_downloaded(os.path.join(FLAGS.download_path, file_name), "ddbcc7dbe423f4249bde10a32d6a7fc9"):
        download(url, FLAGS.download_path, FLAGS.output_path, file_name)
    else:
        print("Already downloaded: {}".format(file_name))


if __name__ == '__main__':
    tf.app.run()