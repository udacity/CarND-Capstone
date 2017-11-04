#Script to generate TF records for Object detection, takes yaml file as an input (Bosch data set)
#Change YAML path in line #63
#To run - python dataPreparation.py --output_path='./'

import tensorflow as tf
from object_detection.utils import dataset_util
import cv2
import yaml
import numpy as np
import os
import pandas as pd
#from __future__ import print_function
import matplotlib
import matplotlib.pyplot as plt
#%matplotlib inline

img_format = b'png'
flags = tf.app.flags
flags.DEFINE_string('output_path', '', 'Path to output TFRecord')
FLAGS = flags.FLAGS

def getTFExample(img,width=1280,height=720,image_format=img_format):
    
    #height = 720
    #width = 1280 
    #filename = images[460]['path']
    
    xmins = list()
    ymins = list()
    ymaxs = list()
    xmaxs = list()
    classes_text = list() 
    classes = list()

    filename = img['path']
    for box in (img['boxes']):
        xmins.append(box['x_min']/width)
        xmaxs.append(box['x_max']/width)
        ymins.append(box['y_min']/height)
        ymaxs.append(box['y_min']/height)
        classes_text.append(box['label'])
        classes.append(class_list.index(box['label'])+1)
    with tf.gfile.GFile(filename, 'rb') as fid:
        encoded_image_data = fid.read()

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

#Read YAML file
#Change the path to train/test yaml file
input_yaml = '../../Bosch-TL-Dataset/data/test.yaml'
riib = False
images = yaml.load(open(input_yaml, 'rb').read())
for i in range(len(images)):
    images[i]['path'] = os.path.abspath(os.path.join(os.path.dirname(input_yaml), images[i]['path']))

#Filter out images without bounding boxes
images_filtered = list()
for i in range(len(images)):
    if len(images[i]['boxes'])>0:
        images_filtered.append(images[i])
del images 

cls_list = list()
for i in range(len(images_filtered)):
    t_img = images_filtered[i]
    t_class = list()
    for j in range(len(t_img['boxes'])):
        t_class.append(t_img['boxes'][j]['label'])
    cls_list.append(t_class)
    
class_list = list(set([item for sublist in cls_list for item in sublist]))
class_idx = list(range(1, len(class_list)+1))
assert len(class_idx)==len(class_list)

def main(_):
    out_path = os.path.join(FLAGS.output_path,'tl_test.record')
    writer = tf.python_io.TFRecordWriter(out_path)
    ctr = 0
    for img in images_filtered:
	ctr+=1
	if ctr%100 == 0:
		print('Processed ', ctr, ' images')
        tf_example = getTFExample(img)
        writer.write(tf_example.SerializeToString())
    writer.close()
 
if __name__ == '__main__':
    #tf.global_variables_initializer()
    tf.app.run()
