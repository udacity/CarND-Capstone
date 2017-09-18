#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf
import math
import csv
import yaml

bridge = CvBridge()

def conv2car_coord(car_pose, g_position):
	l_position = g_position
	
	pos = car_pose.position
	qor = car_pose.orientation
	quaternion = (qor.x, qor.y, qor.z, qor.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	theta = euler[2]
	
	x = (g_position.x - pos.x)*math.cos(theta)+(g_position.y - pos.y)*math.sin(theta)
	y = (g_position.x - pos.x)*(-math.sin(theta))+(g_position.y - pos.y)*math.cos(theta)
	l_position.x = x
	l_position.y = y
	
	return l_position
	
def get_closest_lights(car_pose, lights, light_positions):
	min_ds = 10000000.0
	#min_x = 10000.0
	#min_y = 10000.0
	n = 10
	
	i  = 0
	st = 4
	for light in lights:
		light.pose.pose.position.x = light_positions[i][0]
		light.pose.pose.position.y = light_positions[i][1]
		lp = conv2car_coord(car_pose, light.pose.pose.position)
		st = light.state
		if lp.x > 0:
			ds = math.sqrt(lp.x*lp.x + lp.y*lp.y)
			if ds < min_ds:
				min_ds = ds
				#min_x = lp.x
				#min_y = lp.y
				n = i
		i = i +1
	
	return (n, min_ds, st)
	

def read_bag():
	#print "hello, I am running!"
	bag = rosbag.Bag('sim_drive.bag')
	csvf = open("tldf.csv", "w")
	
	with open("sim_traffic_light_config.yaml", 'r') as stream:
		config = yaml.load(stream)
	#print config
	
	light_positions = config['light_positions']
	#print "light positions:", light_positions
	
	i = 0
	for topic, msg, t in bag.read_messages(topics=['/current_pose', '/image_color', '/vehicle/traffic_lights']):
		print topic
		if topic == '/image_color':
			print 'yes, it is an image'
			cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
			fstr = 'images/'+ 'frame' + str(i) + '.jpeg'
			cv2.imwrite(fstr, cv2_img)
			#print 'car pose is:', car_pose
			tldf = get_closest_lights(car_pose, lights, light_positions)
			print "closest light:", tldf
			tldf_es = fstr+','+str(tldf[0])+','+str(tldf[1])+','+str(tldf[2])+'\n'
			csvf.write(tldf_es)
			i = i +1
		elif topic == '/current_pose':
			car_pose = msg.pose
			print car_pose
			pos = car_pose.position
			qor = car_pose.orientation
			quaternion = (qor.x, qor.y, qor.z, qor.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)
			print 'euler orientation: ', euler[0], euler[1], euler[2]
		elif topic == '/vehicle/traffic_lights':
			lights = msg.lights
			print lights[0].pose, lights[0].state
			#lpos = conv2car_coord(car_pose, lights[0].pose.pose.position)
			#print lpos
			
		#print msg.header
		#i = i +1
		#if i > 50:
		#	break
			
	bag.close()
	csvf.close()

if __name__ == "__main__":
	read_bag()

