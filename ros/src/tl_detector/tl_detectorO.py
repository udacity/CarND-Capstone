#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from PIL import Image as Image1
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import tensorflow as tf1
from tensorflow.contrib.layers import flatten
from scipy.spatial import KDTree
import numpy as np
from PIL import ImageFilter
x = None

STATE_COUNT_THRESHOLD = 3
class MyGaussianBlur(ImageFilter.Filter):
    name = "GaussianBlur"

    def __init__(self, radius=2, bounds=None):
        self.radius = radius
        self.bounds = bounds

    def filter(self, image):
        if self.bounds:
            clips = image.crop(self.bounds).gaussian_blur(self.radius)
            image.paste(clips, self.bounds)
            return image
        else:
            return image.gaussian_blur(self.radius)

def LeNet(x):    
	# Arguments used for tf.truncated_normal, randomly defines variables for the weights and biases for each layer
	mu = 0
	sigma = 0.1
	    
	# TODO: Layer 1: Convolutional. Input = 32x32x1. Output = 28x28x6.
	conv1_w=tf1.Variable(tf1.truncated_normal(shape=(10, 10, 3, 6),mean=mu,stddev=sigma))
	conv1_b=tf1.Variable(tf1.zeros(6))
	conv1=tf1.nn.conv2d(x,conv1_w,strides=[1,4,4,1],padding='VALID')+conv1_b
	# TODO: Activation.
	conv1=tf1.nn.relu(conv1)
	# TODO: Pooling. Input = 28x28x6. Output = 14x14x6.
	conv1=tf1.nn.max_pool(conv1,ksize=[1,2,2,1],strides=[1,2,2,1],padding='VALID')
	# TODO: Layer 2: Convolutional. Output = 10x10x16.
	conv2_w=tf1.Variable(tf1.truncated_normal(shape=(5, 5, 6, 16),mean=mu,stddev=sigma))
	conv2_b=tf1.Variable(tf1.zeros(16))
	conv2=tf1.nn.conv2d(conv1,conv2_w,strides=[1,1,1,1],padding='VALID')+conv2_b
	    
	# TODO: Activation.
	conv2=tf1.nn.relu(conv2)
	# TODO: Pooling. Input = 10x10x16. Output = 5x5x16.
	conv2=tf1.nn.max_pool(conv2,ksize=[1,2,2,1],strides=[1,2,2,1],padding='VALID')

	# TODO: Flatten. Input = 5x5x16. Output = 400.
	fe0=flatten(conv2)
	# TODO: Layer 3: Fully Connected. Input = 400. Output = 120.
	fc1_w=tf1.Variable(tf1.truncated_normal(shape=(400,120),mean=mu,stddev=sigma))
	fc1_b=tf1.Variable(tf1.zeros(120))
	fc1=tf1.matmul(fe0,fc1_w)+fc1_b
	    
	# TODO: Activation.
	fc1=tf1.nn.relu(fc1)
	# TODO: Layer 4: Fully Connected. Input = 120. Output = 84.
	fc2_w=tf1.Variable(tf1.truncated_normal(shape=(120,84),mean=mu,stddev=sigma))
	fc2_b=tf1.Variable(tf1.zeros(84))
	fc2=tf1.matmul(fc1,fc2_w)+fc2_b
	   
	# TODO: Activation.
	fc2=tf1.nn.relu(fc2)
	# TODO: Layer 5: Fully Connected. Input = 84. Output = 10.
	fc3_w=tf1.Variable(tf1.truncated_normal(shape=(84,2),mean=mu,stddev=sigma))
	fc3_b=tf1.Variable(tf1.zeros(2))
	logits_temp=tf1.matmul(fc2,fc3_w)+fc3_b
    
	return logits_temp
logits = None
def classify(imgarray):
	img=imgarray[0].squeeze()
	image=Image1.fromarray(img)
	image=image.filter(MyGaussianBlur(radius=1))
	imgarray[0]=np.array(image)

	global x
	global logits
	if x==None:
		x = tf1.placeholder(tf1.float32, (None, 118, 118, 3))
		logits = LeNet(x)

	sess=tf1.Session()
	saver=tf1.train.Saver()
	saver.restore(sess,'./lenet')
	out=sess.run(tf1.argmax(logits,1),feed_dict={x:imgarray})
	rospy.loginfo('img:%s',out[0])
	return out[0]
def getLightStateR(img):
	gra=cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
	hsv=cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
	h_ch=hsv[:,:,0]
	s_ch=hsv[:,:,1]
	v_ch=hsv[:,:,2]
	sbin=np.ones_like(gra)
	sbin[:,:]=255
	sbin[(h_ch>=10)&(h_ch<=116)]=0
	sbin[(s_ch>=0)&(s_ch<183)]=0
	sbin[(v_ch>=0)&(v_ch<86)]=0
	num=np.sum(sbin)
	if num>20000:
		return True
	else:
		return False
def getLightStateG(img):
	gra=cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
	rgb=img
	r_ch=rgb[:,:,0]
	g_ch=rgb[:,:,1]
	b_ch=rgb[:,:,2]
	sbin=np.ones_like(gra)
	sbin[:,:]=255
	sbin[(r_ch<43)|(r_ch>83)]=0
	sbin[(g_ch<233)|(g_ch>255)]=0
	sbin[(b_ch<43)|(b_ch>83)]=0
	num=np.sum(sbin)
	if num>1000:
		return True
	else:
		return False

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.waypoint_tree=None
        self.lights = []
        self.processnum=0
        self.imagenum=0
	self.sess=None
	self.light_dif=200

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        waypoints_2d=[[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
        self.waypoint_tree=KDTree(waypoints_2d)

    def traffic_cb(self, msg):
        #return
        self.lights = msg.lights

    def image_cb(self, msg):
        #return
        if self.processnum<3:
            self.processnum+=1
            return
        self.processnum=0
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED or state == TrafficLight.YELLOW else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x,y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx=self.waypoint_tree.query([x,y],1)[1]
        return closest_idx
    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
#        return light.state
        if(not self.has_image):
            self.prev_light_loc = None
            return False
	if self.light_dif<50:
		cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
		image=Image1.fromarray(cv_image)
		#self.imagenum+=1
		#image.save('img/'+str(self.imagenum)+'-'+str(light.state)+'aa.jpg')
		calstate=1
		if getLightStateR(cv_image):
			calstate=0
		else:
			if getLightStateG(cv_image):
				calstate=2
		if calstate!=light.state:
			print('light:'+str(calstate)+':'+str(light.state))
		return calstate
		
	else:
		return 2
	
#        if(not self.has_image):
#            self.prev_light_loc = None
#            return False
#	if self.light_dif<50:
#		cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
#		image=Image1.fromarray(cv_image)
#		self.imagenum+=1
#		image.save('img/'+str(self.imagenum)+'-'+str(light.state)+'aa.jpg')
#		#image=Image1.open('img/'+str(self.imagenum)+'-'+str(light.state)+'.jpg')
#		image=image.resize((118,118))	
	       #Get classification
#		imglist=[]
#		imglist.append(np.array(image))
#		imgarray=np.array(imglist)
#		return classify(imgarray)*2
#	else:
#		return 2
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light=None
        line_wp_idx=None
        
#        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx=self.get_closest_waypoint(self.pose.pose.position.x,self.pose.pose.position.y)
            #TODO find the closest visible traffic light (if one exists)
            diff=len(self.waypoints.waypoints)
            for i,light in enumerate(self.lights):
                #get stop line waypoint index
                line=stop_line_positions[i]
                temp_wp_idx=self.get_closest_waypoint(line[0],line[1])
                #find closest stop line waypoint index
                d=temp_wp_idx-car_wp_idx
                if d>=0 and d<diff:
                    diff=d
                    closest_light=light
                    line_wp_idx=temp_wp_idx
        if closest_light:
            state=self.get_light_state(closest_light)
	    self.light_dif=diff
            return line_wp_idx,state
	else:
	    self.light_dif=200
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
