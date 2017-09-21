#!/usr/bin/env python

import math
import rospy

from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane

from pid import PID
from cte import CTE
from dbw_logger import DBWLogger
from yaw_controller import YawController
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

        self.rate = 50  # Hz
        self.sample_time = 1.0 / self.rate

        self.dbw_enabled = True
        self.initialized = False

        self.current_linear_velocity = None
        self.target_linear_velocity = None
        self.target_angular_velocity = None

        self.final_waypoints = None
        self.current_pose = None
        self.logger = DBWLogger(self, rate=1)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        yaw_controller = YawController(wheel_base=wheel_base,
                                       steer_ratio=steer_ratio,
                                       min_speed=0.,
                                       max_lat_accel=max_lat_accel,
                                       max_steer_angle=max_steer_angle)

        self.controller = TwistController(yaw_controller, max_steer_angle, self.sample_time)

        # TODO: Subscribe to all the topics you need to

        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb, queue_size=1)

        self.loop()


    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

    @staticmethod
    def compute_absolute_velocity(velocity_vector):
        x = velocity_vector.x
        y = velocity_vector.y
        z = velocity_vector.z
        return math.sqrt(x**2 + y**2 + z**2)

    def current_velocity_cb(self, msg):
        self.current_linear_velocity = self.compute_absolute_velocity(msg.twist.linear)

    def twist_cmd_cb(self, msg):
        self.target_linear_velocity = self.compute_absolute_velocity(msg.twist.linear)
        self.target_angular_velocity = self.compute_absolute_velocity(msg.twist.angular)

    def final_waypoints_cb(self, msg):
        self.final_waypoints = msg.waypoints

    def current_pose_cb(self, msg):
        self.current_pose = msg.pose

    def loop(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            throttle, brake, steer = 1, None, None

            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)

            # sending None for break, ensures we're not throttling/breaking at the same time

            if (self.final_waypoints is not None) \
                & (self.current_pose is not None) \
                & (self.current_linear_velocity is not None) \
                & (self.target_linear_velocity is not None) \
                & (self.target_angular_velocity is not None):

                self.initialized = True

                cte = CTE.compute_cte(self.final_waypoints, self.current_pose)

                throttle, brake, steer = self.controller.control(self.target_linear_velocity,
                                                                 self.target_angular_velocity,
                                                                 self.current_linear_velocity,
                                                                 cte)

                self.logger.log(throttle, brake, steer, 0, cte)

            if self.initialized and self.dbw_enabled:
                self.publish(throttle, brake, steer)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        if throttle is not None:
            tcmd = ThrottleCmd()
            tcmd.enable = True
            tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            tcmd.pedal_cmd = throttle
            self.throttle_pub.publish(tcmd)

        if brake is not None:
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
            bcmd.pedal_cmd = brake
            self.brake_pub.publish(bcmd)

        if steer is not None:
            scmd = SteeringCmd()
            scmd.enable = True
            scmd.steering_wheel_angle_cmd = steer
            self.steer_pub.publish(scmd)


if __name__ == '__main__':
    DBWNode()
