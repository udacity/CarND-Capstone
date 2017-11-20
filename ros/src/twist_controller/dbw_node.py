#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math

from twist_controller import Controller
from waypoint_lib import helper

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

        min_speed = 0.0
        self.controller = Controller(decel_limit,
                                     accel_limit,
                                     max_steer_angle,
                                     wheel_base,
                                     steer_ratio,
                                     min_speed,
                                     max_lat_accel)

        # Member variables to store current state
        self.current_linear_velocity = None
        self.current_angular_velocity = None
        self.current_pose = None
        self.twist_cmd = None
        self.final_waypoints = None
        self.dbw_enabled = None
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.vehicle_dbw_enabled)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)

        self.loop()

    def all_params_received(self):
        all_params = [self.current_linear_velocity,
                      self.current_angular_velocity,
                      self.current_pose,
                      self.twist_cmd,
                      self.final_waypoints,
                      self.dbw_enabled]
        if None in all_params:
            return False

        return True

    def current_velocity_cb(self, msg):
        rospy.loginfo('Current Velocity Received...')
        self.current_linear_velocity = msg.twist.linear.x
        self.current_angular_velocity = msg.twist.angular.x

    def current_pose_cb(self, msg):
        rospy.loginfo('Current Pose Received...')
        self.current_pose = msg

    def twist_cmd_cb(self, msg):
        rospy.loginfo('Current Twist Received...')
        self.twist_cmd = msg
        self.target_linear_vel = msg.twist.linear.x
        self.target_angular_vel = msg.twist.angular.x

    def final_waypoints_cb(self, msg):
        rospy.loginfo('Final Waypoints Received...')
        self.final_waypoints = msg.waypoints

    def vehicle_dbw_enabled(self, msg):
        rospy.loginfo('DBW Enabled Param Received...')
        self.dbw_enabled = msg

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
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
            if not self.all_params_received():
              rospy.loginfo('Waiting for all params ...')
              continue
            elif self.dbw_enabled:
                steer_cte = helper.calc_steer_cte(self.current_pose, self.final_waypoints)
                throttle, brake, steering = self.controller.control(self.target_linear_vel,
                                                                    self.target_angular_vel,
                                                                    self.current_linear_velocity,
                                                                    self.current_angular_velocity,
                                                                    steer_cte)
                rospy.loginfo('throttle: %s, brake: %s, steering: %s', throttle, brake, steering)
                self.publish(throttle, brake, steering)
            else:
                self.controller.reset()

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
