import cv2
import tensorflow as tf
import glob
import os
from lxml import etree
import random
import io
from PIL import Image


from object_detection.utils import dataset_util

flags = tf.app.flags
flags.DEFINE_string('input_path', '', 'Path to find image files')
flags.DEFINE_string('annotation_path', '', 'Path to find Pascal VOC annotations')
flags.DEFINE_string('output_path', '', 'Path to output TFRecord')
FLAGS = flags.FLAGS

LABEL_MAP = {'green':1,
 'red':2,
 'yellow':3,
 'off':4}

LABELS = ['Green', 'Red', 'Yellow', 'Off']


def create_tf_example(example):
  filename = example['path'].encode()
  # image = cv2.imread(FLAGS.input_path+"/"+filename)
  # encoded_image_data = image.tobytes()
  with tf.gfile.GFile(example['path'], 'rb') as fid:
      encoded_image_data = fid.read()
  encoded_jpg_io = io.BytesIO(encoded_image_data)
  image = Image.open(encoded_jpg_io)
  width, height = image.size

  width = example['width']
  height = example['height'] 
  image_format = 'jpeg'.encode()

  boxes = example['boxes']
  xmins = [b['x_min']/float(width) for b in boxes] # List of normalized left x coordinates in bounding box (1 per box)
  xmaxs = [b['x_max']/float(width) for b in boxes] # List of normalized right x coordinates in bounding box
             # (1 per box)
  ymins = [b['y_min']/float(height) for b in boxes] # List of normalized top y coordinates in bounding box (1 per box)
  ymaxs = [b['y_max']/float(height) for b in boxes] # List of normalized bottom y coordinates in bounding box
             # (1 per box)

  classes = [LABEL_MAP[b['label']] for b in boxes] # List of integer class id of bounding box (1 per box)
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
    images = [os.path.splitext(os.path.basename(f))[0] for f in glob.iglob(FLAGS.input_path + "/*.jpg")]
    examples = []
    for i in images:
        example = {}
        boxes = []
        path = os.path.join(FLAGS.annotation_path, i + '.xml')
        print(i)
        if os.path.exists(path):
            print(path)
            with tf.gfile.GFile(path, 'r') as fid:
                xml_str = fid.read()
                xml = etree.fromstring(xml_str)
                data = dataset_util.recursive_parse_xml_to_dict(xml)['annotation']
		if 'object' in data:
                  for o in data['object']:
                      box = {}
                      box['x_min'] = float(o['bndbox']['xmin'])
                      box['y_min'] = float(o['bndbox']['ymin'])
                      box['x_max'] = float(o['bndbox']['xmax'])
                      box['y_max'] = float(o['bndbox']['ymax'])
                      box['label'] = o['name']
                      boxes.append(box)
                example['path'] = FLAGS.input_path + "/" + i + ".jpg"
                example['boxes'] = boxes
	        example['width'] = int(data['size']['width'])
	        example['height'] = int(data['size']['height'])
                examples.append(example)

    random.shuffle(examples)

    split = int(len(examples) * 0.8)
    print(len(examples))
    create_tf_record(FLAGS.output_path + "/train.tf", examples[:split])
    create_tf_record(FLAGS.output_path + "/valid.tf", examples[split:])


if __name__ == '__main__':
  tf.app.run()
