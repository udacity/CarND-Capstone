import cv2
import tensorflow as tf
import yaml
import random
import io

from PIL import Image

from object_detection.utils import dataset_util


flags = tf.app.flags
flags.DEFINE_string('input_path', '', 'Path to find YAML data')
flags.DEFINE_string('output_path', '', 'Path to output TFRecord')
FLAGS = flags.FLAGS

LABEL_MAP = {'Green':1,
 'Red':2,
 'Yellow':3,
 'Other':4}

LABELS = ['Green', 'Red', 'Yellow', 'Other']


def create_tf_example(example):
  filename = example['filename'].encode()
  with tf.gfile.GFile(FLAGS.input_path+"/"+example['filename'], 'rb') as fid:
      encoded_image_data = fid.read()
  encoded_jpg_io = io.BytesIO(encoded_image_data)
  image = Image.open(encoded_jpg_io)
  width, height = image.size
  image_format = 'jpg'.encode()

  boxes = example['annotations']
  xmins = [b['xmin']/float(width) for b in boxes] # List of normalized left x coordinates in bounding box (1 per box)
  xmaxs = [(b['xmin']+b['x_width'])/float(width) for b in boxes] # List of normalized right x coordinates in bounding box
             # (1 per box)
  ymins = [b['ymin']/float(height) for b in boxes] # List of normalized top y coordinates in bounding box (1 per box)
  ymaxs = [(b['ymin']+b['y_height'])/float(height) for b in boxes] # List of normalized bottom y coordinates in bounding box
             # (1 per box)

  classes = [LABEL_MAP[b['class']] for b in boxes] # List of integer class id of bounding box (1 per box)
  classes_text = [LABELS[c-1] for c in classes] # List of string class name of bounding box (1 per box)

  tf_example = tf.train.Example(features=tf.train.Features(feature={
      'image/height': dataset_util.int64_feature(height),
      'image/width': dataset_util.int64_feature(width),
      'image/filename': dataset_util.bytes_feature(filename),
      'image/source_id': dataset_util.bytes_feature(filename),
      'image/encoded': dataset_util.bytes_feature(encoded_image_data),
      'image/format': dataset_util.bytes_feature(image_format),
      'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
      'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
      'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
      'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
      'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
      'image/object/class/label': dataset_util.int64_list_feature(classes),
  }))
  return tf_example

def create_tf_record(path, examples):
    counter = 0
    writer = tf.python_io.TFRecordWriter(path)
    print("Total ", len(examples))
    for example in examples:
        counter += 1
        if counter % 10 == 0:
            print("Completed ", (float(counter)/len(examples))*100, "%")
        tf_example = create_tf_example(example)
        writer.write(tf_example.SerializeToString())
    writer.close()

def main(_):

  yaml_file = FLAGS.input_path + "/sim_data_large.yaml"
  with open(yaml_file) as f:
      examples = yaml.load(f)
      random.shuffle(examples)

      split = int(len(examples)*1.0)
      create_tf_record(FLAGS.output_path + "/train.tf", examples[:split])
      create_tf_record(FLAGS.output_path + "/valid.tf", examples[split:])





if __name__ == '__main__':
  tf.app.run()
