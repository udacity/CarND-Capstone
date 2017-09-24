#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller
from yaw_controller import YawController
from lowpass import LowPassFilter

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''
def twist_to_xyy(msg):
    x = msg.twist.linear.x
    y = msg.twist.linear.y
    yaw = msg.twist.angular.z
    return(x,y,yaw)


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')
        rospy.loginfo("initing dbw_node")

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.dbw = False
        self.angular_velocity_filter = LowPassFilter(.90, 1)
        self.velocity_filter = LowPassFilter(.90, 1)
        self.twist_yaw_filter = LowPassFilter(.96, 1)
        self.twist_velocity_filter = LowPassFilter(.96, .8)
        min_speed = .1
        # At high speeds, a multiple of 1.2 seems to work a bit
        # better than 1.0
        self.yaw_controller = YawController(wheel_base, 1.2*steer_ratio, min_speed, 8*max_lat_accel, max_steer_angle)

        self.loop()

    '''
    /twist_cmd
    Type: geometry_msgs/TwistStamped
    geometry_msgs/TwistStampedstd_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Twist twist
      geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z
    '''



    def twist_cmd_cb(self, msg):
        seq = msg.header.seq
        (x, y, yaw) = twist_to_xyy(msg)
        # rospy.loginfo("twist_cmd_cb %d", seq)
        self.twist_yaw_filter.filt(yaw)
        self.twist_velocity_filter.filt(x)
        if msg.header.seq%5 == 0:
            ts = msg.header.stamp.secs + 1.e-9*msg.header.stamp.nsecs
            # rospy.loginfo("tcc %d %f %f  %f %f", seq, ts, x, yaw, self.twist_yaw_filter.get())
        pass

    '''
    /vehicle/dbw_enabled
    Type: std_msgs/Bool
    '''
    def dbw_enabled_cb(self, msg):
        rospy.loginfo("dbw_enabled_cb %s", msg)
        self.dbw = msg

    '''
    /current_velocity
    Type: geometry_msgs/TwistStamped
    '''
    def current_velocity_cb(self, msg):
        seq = msg.header.seq
        (x, y, yaw) = twist_to_xyy(msg)
        # rospy.loginfo("current_velocity_cb %i", seq)
        # factor = .90
        # if yaw != 0.:
        # self.filtered_angular_velocity = factor*self.filtered_angular_velocity + (1-factor)*yaw
        # self.filtered_velocity = factor*self.filtered_velocity + (1-factor)*x
        self.angular_velocity_filter.filt(yaw)
        self.velocity_filter.filt(x)

        if msg.header.seq%5 == 0:
            ts = msg.header.stamp.secs + 1.e-9*msg.header.stamp.nsecs
            # ts seems to always be 0.
            yaw_per_meter = self.angular_velocity_filter.get()/self.velocity_filter.get()
            # rospy.loginfo("cv %d  %f %f  %f %f  %f", seq, self.velocity_filter.get(), x, self.angular_velocity_filter.get(), yaw, yaw_per_meter)
        pass


    def loop(self):
        # rate = rospy.Rate(50) # 50Hz
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)
            twist = self.twist_yaw_filter.get()
            steer = self.yaw_controller.get_steering(self.twist_velocity_filter.get(), twist, self.velocity_filter.get())
            # rospy.loginfo("steering angle %f (twist %f)", steer, twist)
            # throttle is 0.35, which runs the car at about 40 mph.
            # throttle of 0.98 will run the car at about 115 mph.
            self.publish(0.35,0.,steer)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
