#!/usr/bin/python

import os
import glob
import shutil
from sys import argv
import yaml
import csv
import xml.etree.ElementTree as ET
from lxml import etree
from lxml import builder
from PIL import Image

data_root_1 = '/media/peng/Data/ROS_yuan/CarND-Capstone/ros/src/yolo_light/scripts/Data/dataset-sdcnd-capstone/data/real_training_data/'
data_root_2 = '/media/peng/Data/ROS_yuan/CarND-Capstone/ros/src/yolo_light/scripts/Data/dataset-sdcnd-capstone/data/sim_training_data/'
data_root_3 = '/media/peng/Data/ROS_yuan/CarND-Capstone/ros/src/yolo_light/scripts/Data/object-dataset/'

anno_1 = 'real_data_annotations.yaml'
anno_2 = 'sim_data_annotations.yaml'
anno_3 = 'labels.csv'

out_root = '/media/peng/Data/ROS_yuan/CarND-Capstone/ros/src/yolo_light/scripts/'
if not os.path.isdir(out_root): os.mkdir(out_root)
if not os.path.isdir(out_root+'Data/'): os.mkdir(out_root+'Data/')
if not os.path.isdir(out_root+'Annotations/'): os.mkdir(out_root+'Annotations/')

num_channels = 3

def parse_yaml(root, anno, count):
	with open(root+anno, 'r') as stream:
		anno_data = yaml.load(stream)

	for i in anno_data:
		# reallocate images
		img_name = root + i['filename']
		new_img_name = '{}.jpg'.format(format(count, '06d'))
		shutil.copy(img_name, out_root+'Data/'+new_img_name)

		# convert annotations and construct xml
		annotation = etree.Element('annotation')
		folder = etree.SubElement(annotation, 'folder')
		folder.text = 'CarND-Capstone'
		
		filename = etree.SubElement(annotation, 'filename')
		filename.text = new_img_name

		# get size
		img = Image.open(img_name) # this does not load the image, default #channels = 3
		size = etree.SubElement(annotation, 'size')
		width = etree.SubElement(size, 'width')
		width.text = str(img.size[0])
		height = etree.SubElement(size, 'height')
		height.text = str(img.size[1])
		depth = etree.SubElement(size, 'depth')
		depth.text = str(num_channels)

		# get objects
		for a in i['annotations']:	 
			obj = etree.SubElement(annotation, 'object')
			name = etree.SubElement(obj, 'name')
			name.text = a['class'].lower()
			bndbox = etree.SubElement(obj, 'bndbox')
			xmin = etree.SubElement(bndbox, 'xmin')
			xmin.text = str(int(round(a['xmin'])))
			ymin = etree.SubElement(bndbox, 'ymin')
			ymin.text = str(int(round(a['ymin'])))
			xmax = etree.SubElement(bndbox, 'xmax')
			xmax.text = str(int(round(a['xmin'] + a['x_width'])))
			ymax = etree.SubElement(bndbox, 'ymax')
			ymax.text = str(int(round(a['ymin'] + a['y_height'])))

		tree = etree.ElementTree(annotation)

		xml_name = '{}.xml'.format(format(count, '06d'))
		tree.write(out_root+'Annotations/'+xml_name, pretty_print=True, xml_declaration=False, encoding="utf-8")

		count += 1
		if count % 100 == 0: print('Processed:', count)

	return anno_data, count

def parse_csv(root, anno, count):
	with open(root+anno, 'rb') as f:
		anno_data = list(csv.reader(f))

	img_name_prev = ''
	for i in anno_data:
		line = i[0].split(' ')

		if len(line) > 7: # has traffic light
			class_name = line[7].replace('"', '').lower()
			if class_name[-4:] == 'left':
				class_name = class_name[:-4]

			if line[0] != img_name_prev:
				# reallocate images
				img_name = root + line[0]
				new_img_name = '{}.jpg'.format(format(count, '06d'))
				shutil.copy(img_name, out_root+'Data/'+new_img_name)

				# convert annotations and construct xml
				annotation = etree.Element('annotation')
				folder = etree.SubElement(annotation, 'folder')
				folder.text = 'CarND-Capstone'
				
				filename = etree.SubElement(annotation, 'filename')
				filename.text = new_img_name

				# get size
				img = Image.open(img_name) # this does not load the image, default #channels = 3
				size = etree.SubElement(annotation, 'size')
				width = etree.SubElement(size, 'width')
				width.text = str(img.size[0])
				height = etree.SubElement(size, 'height')
				height.text = str(img.size[1])
				depth = etree.SubElement(size, 'depth')
				depth.text = str(num_channels)

				xml_name = '{}.xml'.format(format(count, '06d'))

				count += 1
				if count % 100 == 0: print('Processed:', count)

			# get object
			obj = etree.SubElement(annotation, 'object')
			name = etree.SubElement(obj, 'name')
			name.text = class_name
			bndbox = etree.SubElement(obj, 'bndbox')
			xmin = etree.SubElement(bndbox, 'xmin')
			xmin.text = str(int(line[1]))
			ymin = etree.SubElement(bndbox, 'ymin')
			ymin.text = str(int(line[2]))
			xmax = etree.SubElement(bndbox, 'xmax')
			xmax.text = str(int(line[3]))
			ymax = etree.SubElement(bndbox, 'ymax')
			ymax.text = str(int(line[4]))

			tree = etree.ElementTree(annotation)

			tree.write(out_root+'Annotations/'+xml_name, pretty_print=True, xml_declaration=False, encoding="utf-8")

			img_name_prev = line[0]

	return anno_data, count

def sort_data():	
	count = 0

	_, count = parse_yaml(data_root_1, anno_1, count)
	_, count = parse_yaml(data_root_2, anno_2, count)
	_, count = parse_csv(data_root_3, anno_3, count)

	print('Processed total:', count)

if __name__ == '__main__':
	sort_data()
