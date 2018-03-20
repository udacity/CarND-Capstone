#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math
import csv

from twist_controller import TwistController

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

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

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

        self.dbw_enabled = False
        self.target_velocity = 0.
        self.target_yaw_dot = 0.
        self.current_velocity = 0.
        self.current_yaw_dot = 0.

        self.last_update_time = None

        # Initialize TwistController
        self.twist_controller = TwistController(accel_limit, -1., max_steer_angle, BrakeCmd.TORQUE_MAX,
                                                wheel_base, steer_ratio, max_lat_accel, max_steer_angle)

        # Subscribe to topics
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_callback)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_callback)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_callback)


        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():

            if self.last_update_time is None:
                self.last_update_time = rospy.Time.now()
                rate.sleep()
                continue

            current_time = rospy.Time.now()
            dt = current_time.to_sec() - self.last_update_time.to_sec()
            self.last_update_time = current_time

            if (dt>0.075):
                rospy.logwarn('slow DBW update, dt:%.3fs freq:%.1fhz', dt, 1./dt)

            # reset controller PIDs and skip calculations if DBW is off
            if not self.dbw_enabled:
                self.twist_controller.reset()
                rate.sleep()
                continue

            accel, steer = self.twist_controller.control(self.target_velocity,
                                                         self.target_yaw_dot,
                                                         self.current_velocity,
                                                         dt)

            throttle = 0
            brake = 0

            if (accel<0):
                brake = -accel
            else:
                throttle = accel

            # record data for debugging
            # self.data_recorder(self.target_velocity, self.target_yaw_dot, throttle, brake, steer)

            # rospy.loginfo('DBW a:%.3f         y:%.3f', self.target_velocity, self.target_yaw_dot)
            # rospy.loginfo('DBW t:%.3f b:%.3f s:%.3f', throttle, brake, steer)

            self.publish(throttle, brake, steer)
            rate.sleep()

    def twist_cmd_callback(self, msg):
        self.target_velocity = msg.twist.linear.x
        self.target_yaw_dot = msg.twist.angular.z
        # rospy.loginfo('twist_cmd: v:%.3f yd:%.3f', self.target_velocity, self.target_yaw_dot)
        # log_twist_msg(msg, 'twist_cmd:')

    def current_velocity_callback(self, msg):
        self.current_velocity = msg.twist.linear.x
        self.current_yaw_dot = msg.twist.angular.z
        # rospy.loginfo('current_velocity: v:%.3f yd:%.3f', self.current_velocity, self.current_yaw_dot)
        # log_twist_msg(msg, 'current_velocity:')

    def dbw_enabled_callback(self, msg):
        self.dbw_enabled = msg.data
        # rospy.loginfo('dbw_enabled: %d', self.dbw_enabled)

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


    def data_recorder(self, target_velocity, target_yaw_dot, throttle, brake, steer):
        dbw_data_filename='/tmp/dbw_data.csv'
        try:
            foo = self.dbw_data
        except:
            self.dbw_data = []
            with open(dbw_data_filename, 'wb') as csvfile:
                rospy.loginfo('DBW data file: %s', csvfile.name)
                csv_writer = csv.writer(csvfile, delimiter=',')
                csv_writer.writerow(['time','t_speed', 't_yd', 'throttle', 'brake', 'steer', 'c_speed', 'c_yd'])

        time = rospy.Time.now().to_sec()
        self.dbw_data.append([time, target_velocity, target_yaw_dot, throttle, brake, steer, self.current_velocity, self.current_yaw_dot])
        if len(self.dbw_data)==1000:
            with open(dbw_data_filename, 'ab') as csvfile:
                csv_writer = csv.writer(csvfile, delimiter=',')
                for row in self.dbw_data:
                    csv_writer.writerow(row)
            rospy.loginfo('DBW data saved')
            self.dbw_data = []


def log_twist_msg(msg, description=None):
    if description is None:
        description = ''
    else:
        description += ' '
    rospy.loginfo(description + 'linear: [%.3f, %.3f, %.3f] angular: [%.3f, %.3f, %.3f]',
                   msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                   msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z,)

if __name__ == '__main__':
    DBWNode()
