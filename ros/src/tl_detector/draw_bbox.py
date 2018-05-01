#!/usr/bin/python

import os
import glob
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image
import numpy as np
import json
import random
import xml.etree.ElementTree as ET

data_root = '/mnt/data/datasets/CarND-Capstone/'
viz_num = 10

if __name__ == '__main__':

	img_lst = glob.glob(data_root+'Data/*.jpg')
	lbl_lst = glob.glob(data_root+'Annotations/*.xml')

	img_num = len(img_lst)
	print('Number of images in total: ' + str(img_num))
	print('Number of images to visualize: ' + str(viz_num))

	if (viz_num > img_num):
		raise ValueError('Images to visualize cannot be more than the total images.')

	out_root = data_root + 'annotated/'
	if not os.path.isdir(out_root):
		os.mkdir(out_root)

	iperm = random.sample(range(img_num), viz_num)
	for i in iperm:
		print('Annotating random image #' + str(i))
		name = img_lst[i]
		img = np.array(Image.open(name), dtype=np.uint8)

		# Create figure and axes
		fig, ax = plt.subplots(1)

		# Display the image
		ax.imshow(img)

		# Parse xml
		tree = ET.parse(lbl_lst[i])
		anno_data = tree.getroot()

		for a in anno_data:
			if a.tag == 'object':
				class_name = a[0].text
				x = int(a[1][0].text)
				y = int(a[1][1].text)
				w = int(a[1][2].text) - x
				h = int(a[1][3].text) - y

				# print(x, y, w, h)

				rect = patches.Rectangle((x, y), w, h, linewidth=.5, edgecolor=class_name, facecolor='none')
				ax.text(x, y - 4, class_name, color=class_name, fontsize=4)

				# Add the patch to the Axes
				ax.add_patch(rect)

		out_name = '{}.jpg'.format(format(i, '06d'))
		fig.savefig(out_root+out_name, dpi=200, bbox_inches='tight')
				
		plt.close()
		