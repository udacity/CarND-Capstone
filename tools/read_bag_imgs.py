"""
Read a bag file, extract images from a specific topic, and dump
them into a folder
"""

import argparse
import rospy
import rosbag
import cv2
from os import path
from cv_bridge import CvBridge, CvBridgeError
import tqdm

def main():
	## arguments
	parser = argparse.ArgumentParser()
	parser.add_argument("bagfile", help="source bagfile")
	parser.add_argument("topic", help="name of image topic to extract from")
	parser.add_argument("imgdir", help="path to folder where images will be extracted to")

	args = parser.parse_args()
	bagfile = args.bagfile
	topic = args.topic
	imgdir = args.imgdir

	if not path.isfile(bagfile):
		print("BAG FILE %s doesn't exist" % bagfile)
		exit(-1)
	if not path.isdir(imgdir):
		print("IMG DIR %s doesn't exist" % imgdir)
		exit(-1)

	## read bag file
	bag = rosbag.Bag(bagfile)
	bridge = CvBridge()
	for i, (topic, msg, t) in tqdm.tqdm(enumerate(bag.read_messages(topics=[topic]))):
		encoding = msg.encoding
		img = bridge.imgmsg_to_cv2(msg, encoding)
		img = cv2.cvtColor(img, cv2.COLOR_BAYER_GR2RGB)
		img_name = path.join(imgdir, "%09i.png" % i)
		cv2.imwrite(img_name, img)

if __name__ == '__main__':
	main()