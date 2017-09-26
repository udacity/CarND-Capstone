import cv2
import rosbag
from cv_bridge import CvBridge, CvBridgeError


bag_name = 'udacity_succesful_light_detection.bag' 
out_dir = './imgs/'

bridge = CvBridge()

with rosbag.Bag(bag_name, 'r') as bag:
    for topic, msg, ts in bag.read_messages():
        if topic == '/image_color':
            img_out_name = out_dir+str(ts.secs)+str(ts.nsecs)+'.jpg'
            image = bridge.imgmsg_to_cv2(msg, 'bgr8')
            (b,g,r) = cv2.split(image)
            cv2.imwrite(img_out_name, cv2.merge([b, g, r]))


