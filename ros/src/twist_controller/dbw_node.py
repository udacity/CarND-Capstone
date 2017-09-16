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

        # alternative: kp=1.0, ki=0.001, kd=0.5 - more jiggle, but mostly stays inside the lane
        self.steer_pid_ctrl = PID(kp=0.7, ki=0.004, kd=0.3, mn=-8.0, mx=8.0)  # sometimes leaves lane
        self.steer_yaw_ctrl = YawController(wheel_base, steer_ratio, 0.0, max_lat_accel, max_steer_angle)

        self.dbw_enabled = True
        self.current_linear_velocity = 0.0
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.final_waypoints = None
        self.current_pose = None
        self.logger = DBWLogger(self, rate=1)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)

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

            # todo: control throttle with PID controller
            if self.current_linear_velocity >= self.target_linear_velocity:
                throttle = 0.0

            cte = 0.0
            yaw_steer = 0.0

            if self.final_waypoints is not None and self.current_pose is not None:
                cte = CTE.compute_cte(self.final_waypoints, self.current_pose)
                steer = self.steer_pid_ctrl.step(error=cte, sample_time=self.sample_time)

                yaw_steer = self.steer_yaw_ctrl.get_steering(
                    self.target_linear_velocity,
                    self.target_angular_velocity,
                    self.current_linear_velocity)

                steer = steer + yaw_steer

            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)

            self.logger.log(throttle, brake, steer, yaw_steer, cte)

            # sending None for break, ensures we're not throttling/breaking at the same time
            if self.dbw_enabled:
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
