#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

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

MPH_to_MPS = 1609.344/3600.0 # 1 mile = 1609.344 1 hour = 3600 seconds

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        self.min_speed = rospy.get_param('~min_speed', 4.*0.44704)
        self.max_throttle_percentage = rospy.get_param('~max_throttle_percentage', 0.1)
        self.max_braking_percentage = rospy.get_param('~max_braking_percentage', -0.1)
        self.max_vel_mps = rospy.get_param('waypoint_loader/velocity')*MPH_to_MPS
        self.loop_freq = rospy.get_param('~loop_freq', 2)
        # the frequency to process vehicle messages

        self.desired_velocity, self.current_velocity = None, None

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # DONE: Create `TwistController` object

        self.controller = TwistController(vehicle_mass=self.vehicle_mass,
                                          fuel_capacity=self.fuel_capacity,
                                          brake_deadband=self.brake_deadband,
                                          decel_limit=self.decel_limit,
                                          accel_limit=self.accel_limit,
                                          wheel_radius=self.wheel_radius,
                                          wheel_base=self.wheel_base,
                                          steer_ratio=self.steer_ratio,
                                          max_lat_accel=self.max_lat_accel,
                                          max_steer_angle=self.max_steer_angle)
                                          # min_speed=self.min_speed,
                                          # ,
                                          # max_braking_percentage=self.max_braking_percentage,
                                          # max_throttle_percentage=self.max_throttle_percentage,
                                          # max_vel_mps=self.max_vel_mps

        # DONE: Subscribe to all the topics you need to
        self.dbw_enabled = False
        self.current_velocity = None
        self.desired_velocity = None

        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(self.loop_freq) # from 50Hz to 2Hz
        dt = 1. / self.loop_freq
        while not rospy.is_shutdown():
            if self.desired_velocity and self.current_velocity:
                # DONE: Get predicted throttle, brake, and steering using `twist_controller`
                # You should only publish the control commands if dbw is enabled
                if self.dbw_enabled:
                    throttle, brake, steering = self.controller.control(self.desired_velocity,
                                                                        self.current_velocity,
                                                                        dt)
                    self.publish(throttle, brake, steering)
                # end of self.dbw_enabled
                self.desired_velocity, self.current_velocity = None, None
            # end of if self.desired_velocity and self.current_velocity
            rate.sleep()

    def current_velocity_cb(self, msg):
      if self.current_velocity is None:
          self.current_velocity = msg # .twist
      # end of if self.current_velocity is None:
      # self.current_velocity = msg.twist
    
    def twist_cmd_cb(self, msg):
      self.desired_velocity = msg # .twist
    
    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

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
