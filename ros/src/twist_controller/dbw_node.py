#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

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
        self.target_acceleration = 0.
        self.target_yaw_dot = 0.
        self.current_velocity_x = 0.
        self.current_yaw_dot = 0.

        self.last_update_time = None

        # TODO: Create `Controller` object
        # self.controller = Controller(<Arguments you wish to provide>)

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
            #rospy.loginfo('dbw dt: %.3f', dt)
            self.last_update_time = current_time

            if (dt>0.075):
                rospy.logwarn('slow DBW update, dt:%.3fs freq:%.1fhz', dt, 1/dt)



            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)

            throttle=0.1
            brake=0
            steer=0
            #rospy.loginfo('DBW t:%.3f b:%.3f s:%.3f', throttle, brake, steer)

            if self.dbw_enabled:
                self.publish(throttle, brake, steer)
            rate.sleep()

    def twist_cmd_callback(self, msg):
        self.target_acceleration = msg.twist.linear.x
        self.target_yaw_dot = msg.twist.angular.z
        # rospy.loginfo('twist_cmd: a:%.3f yd:%.3f', self.target_acceleration, self.target_yaw_dot)
        # log_twist_msg(msg, 'twist_cmd:')

    def current_velocity_callback(self, msg):
        self.current_velocity_x = msg.twist.linear.x
        self.current_yaw_dot = msg.twist.angular.z
        # rospy.loginfo('current_velocity: v:%.3f yd:%.3f', self.current_velocity_x, self.current_yaw_dot)
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
