
import rospy

import tf
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from dbw_mkz_msgs.msg import SteeringReport, ThrottleCmd, BrakeCmd, SteeringCmd
from std_msgs.msg import Float32 as Float
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

from styx_msgs.msg import TrafficLight, TrafficLightArray
import numpy as np
from PIL import Image as PIL_Image
from io import BytesIO
import base64

import math

TYPE = {
    'bool': Bool,
    'float': Float,
    'pose': PoseStamped,
    'pcl': PointCloud2,
    'twist': TwistStamped,
    'steer': SteeringReport,
    'trafficlights': TrafficLightArray,
    'steer_cmd': SteeringCmd,
    'brake_cmd': BrakeCmd,
    'throttle_cmd': ThrottleCmd,
    'image':Image
}


class Bridge(object):
    def __init__(self, conf, server):
        rospy.init_node('styx_server')
        self.server = server
        self.vel = 0.
        self.yaw = None
        self.angular_vel = 0.
        self.bridge = CvBridge()

        self.callbacks = {
            '/vehicle/steering_cmd': self.callback_steering,
            '/vehicle/throttle_cmd': self.callback_throttle,
            '/vehicle/brake_cmd': self.callback_brake,
        }

        self.subscribers = [rospy.Subscriber(e.topic, TYPE[e.type], self.callbacks[e.topic])
                            for e in conf.subscribers]

        self.publishers = {e.name: rospy.Publisher(e.topic, TYPE[e.type], queue_size=1)
                           for e in conf.publishers}

    def create_light(self, x, y, z, yaw, state):
        light = TrafficLight()

        light.header = Header()
        light.header.stamp = rospy.Time.now()
        light.header.frame_id = '/world'

        light.pose = self.create_pose(x, y, z, yaw)
        light.state = state

        return light

    def create_pose(self, x, y, z, yaw=0.):
        pose = PoseStamped()

        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = '/world'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        q = tf.transformations.quaternion_from_euler(0., 0., math.pi * yaw/180.)
        pose.pose.orientation = Quaternion(*q)

        return pose

    def create_float(self, val):
        fl = Float()
        fl.data = val
        return fl

    def create_twist(self, velocity, angular):
        tw = TwistStamped()
        tw.twist.linear.x = velocity
        tw.twist.angular.z = angular
        return tw

    def create_steer(self, val):
        st = SteeringReport()
        st.steering_wheel_angle_cmd = val * math.pi/180.
        st.enabled = True
        st.speed = self.vel
        return st

    def calc_angular(self, yaw):
        angular_vel = 0.
        if self.yaw is not None:
            angular_vel = (yaw - self.yaw)/(rospy.get_time() - self.prev_time)
        self.yaw = yaw
        self.prev_time = rospy.get_time()
        return angular_vel

    def create_point_cloud_message(self, pts):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = '/world'
        cloud_message = pcl2.create_cloud_xyz32(header, pts)
        return cloud_message

    def broadcast_transform(self, name, position, orientation):
        br = tf.TransformBroadcaster()
        br.sendTransform(position,
            orientation,
            rospy.Time.now(),
            name,
            "world")

    def publish_odometry(self, data):
        pose = self.create_pose(data['x'], data['y'], data['z'], data['yaw'])

        position = (data['x'], data['y'], data['z'])
        orientation = tf.transformations.quaternion_from_euler(0, 0, math.pi * data['yaw']/180.)
        self.broadcast_transform("base_link", position, orientation)

        self.publishers['current_pose'].publish(pose)
        self.vel = data['velocity']* 0.44704
        self.angular = self.calc_angular(data['yaw'] * math.pi/180.)
        self.publishers['current_velocity'].publish(self.create_twist(self.vel, self.angular))


    def publish_controls(self, data):
        steering, throttle, brake = data['steering_angle'], data['throttle'], data['brake']
        self.publishers['steering_report'].publish(self.create_steer(steering))
        self.publishers['throttle_report'].publish(self.create_float(throttle))
        self.publishers['brake_report'].publish(self.create_float(brake))

    def publish_obstacles(self, data):
        for obs in data['obstacles']:
            pose = self.create_pose(obs[0], obs[1], obs[2])
            self.publishers['obstacle'].publish(pose)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = '/world'
        cloud = pcl2.create_cloud_xyz32(header, data['obstacles'])
        self.publishers['obstacle_points'].publish(cloud)

    def publish_lidar(self, data):
        self.publishers['lidar'].publish(self.create_point_cloud_message(zip(data['lidar_x'], data['lidar_y'], data['lidar_z'])))

    def publish_traffic(self, data):
        x, y, z = data['light_pos_x'], data['light_pos_y'], data['light_pos_z'],
        yaw = [math.atan2(dy, dx) for dx, dy in zip(data['light_pos_dx'], data['light_pos_dy'])]
        status = data['light_state']

        lights = TrafficLightArray()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = '/world'
        lights.lights = [self.create_light(*e) for e in zip(x, y, z, yaw, status)]
        self.publishers['trafficlights'].publish(lights)

    def publish_dbw_status(self, data):
        self.publishers['dbw_status'].publish(Bool(data))

    def publish_camera(self, data):
        imgString = data["image"]
        image = PIL_Image.open(BytesIO(base64.b64decode(imgString)))
        image_array = np.asarray(image)

        image_message = self.bridge.cv2_to_imgmsg(image_array, encoding="rgb8")
        self.publishers['image'].publish(image_message)

    def callback_steering(self, data):
        self.server('steer', data={'steering_angle': str(data.steering_wheel_angle_cmd)})

    def callback_throttle(self, data):
        self.server('throttle', data={'throttle': str(data.pedal_cmd)})

    def callback_brake(self, data):
        self.server('brake', data={'brake': str(data.pedal_cmd)})
