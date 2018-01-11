#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, Twist
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
        '''
        Drive By Wire (DBW) ROS node constructor
        '''

        rospy.init_node('dbw_node')

        # Read some vehicle parameters
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

        # Publishers for steering, throttle and brake commands
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

# Dictionary of parameters sed by the controller
        params = {
            'vehicle_mass': vehicle_mass,
            'fuel_capacity': fuel_capacity,
            'brake_deadband': brake_deadband,
            'decel_limit': decel_limit,
            'accel_limit': accel_limit,
            'wheel_radius': wheel_radius,
            'wheel_base': wheel_base,
            'steer_ratio': steer_ratio,
            'max_lat_accel': max_lat_accel,
            'max_steer_angle': max_steer_angle
        }

        # Create the controller
        self.controller = Controller(**params)

        # Subscribe to current velocity, twist command and drive by wire enabled topics
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_callback)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_callback)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_callback)

        # Initialize current velocity, twist command and drive by wire enabled
        self.current_velocity = Twist()
        self.twist_cmd = Twist()
        self.dbw_enabled = Bool()

        # Enter loop to process topic messages
        self.loop()

    def loop(self):
        '''
        Loop to process messages coming from '/current_velocity',
        '/twist_cmd' and '/vehicle/dbw_enabled' topics.
        @return: None
        '''

        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            throttle, brake, steering = self.controller.control(
                self.twist_cmd.linear.x,
                self.twist_cmd.angular.z,
                self.current_velocity.linear.x,
                self.current_velocity.angular.z)

            # Publish commands only in 'autonomous mode'
            if self.dbw_enabled:
                self.publish(throttle, brake, steering)

            rate.sleep()


    def current_velocity_callback(self, msg):
        '''
        Current velocity topic ('/current_velocity') callback
        @param: msg: the message read from the '/current_velocity' topic
        @return: None
        '''

        self.current_velocity = msg.twist


    def twist_cmd_callback(self, msg):
        '''
        Twist command topic ('/twist_cmd') callback
        @param: msg: the message read from the '/twist_cmd' topic
        @return: None
        '''

        self.twist_cmd = msg.twist


    def dbw_enabled_callback(self, msg):
        '''
        Drive by wire command topic ('/vehicle/dbw_enabled') callback
        @param: msg: the message read from the '/vehicle/dbw_enabled' topic
        @retrun: None
        '''

        self.dbw_enabled = msg.data


    def publish(self, throttle, brake, steer):
        '''
        Publishes the throttle, brake and steer commands to them
        to '/vehicle/throttle_cmd', '/vehicle/brake_cmd' and
        '/vehicle/steering_cmd'
        @param: throttle: the throttle command
        @param: brake: the brake command
        @param: steer: the steering command
        @return: None
        '''

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
