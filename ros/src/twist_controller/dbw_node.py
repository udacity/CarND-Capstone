#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane
import waypoint_lib.helper as helper
import math
import tf

from twist_controller import Controller
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
        min_speed = 0.0

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object <Arguments you wish to provide>
        self.controller = Controller(decel_limit, accel_limit, max_steer_angle,
            max_lat_accel, min_speed, wheel_base, steer_ratio, vehicle_mass, wheel_radius)
        # self.yaw_controller = YawController(wheel_base, steer_ratio, 0.0, max_lat_accel, max_steer_angle)

        self.dbw_enabled = None
        self.current_velocity = None
        self.twist_cmd = None
        self.pose = None
        self.final_waypoints = None


        self.prev_clk = rospy.get_rostime().nsecs
        self.prev_clk_ready = False

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/current_velocity', TwistStamped, self.curr_vel_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb, queue_size=1)


        self.loop()

    def required_all(self):
      required = [self.dbw_enabled, self.current_velocity, self.twist_cmd, self.pose, self.final_waypoints]
      return all([p is not None for p in required])

    def curr_vel_cb(self, curr_vel_msg):
    #   rospy.loginfo("current_velocity = {}".format(curr_vel_msg.twist))
      self.current_velocity = curr_vel_msg.twist

    def twist_cmd_cb(self, twist_cmd_msg):
    #   rospy.loginfo("twist_cmd = {}".format(twist_cmd_msg.twist))
      self.twist_cmd = twist_cmd_msg.twist

    def dbw_enabled_cb(self, dbw_enabled):
    #   rospy.loginfo("dbw_enabled = {}".format(dbw_enabled.data))
      self.dbw_enabled = dbw_enabled.data

    def final_waypoints_cb(self, final_waypoints):
        self.final_waypoints = final_waypoints.waypoints

    def pose_cb(self, pose):
        # rospy.loginfo("pose = {}".format(pose.pose))
        self.pose = pose

        # rospy.loginfo("pose x, y, yaw = {}, {}, {}".format(self.pose.position.x,
        #     self.pose.position.y, helper.yaw_from_orientation(self.pose.orientation)))

    def loop(self):
        rate = rospy.Rate(50) # 50Hz

        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled

            if not self.required_all():
              rospy.loginfo('Waiting for all params ...')
              continue

            # TODO: Move clock to pid controllers
            clk = rospy.get_rostime()
            if not self.prev_clk_ready:
                self.prev_clk_ready = True
                continue

            # Yes, I now it will be always 0.02 but in case someone will change rate ...
            delta_t = (clk.nsecs - self.prev_clk)*1e-9
            # rospy.loginfo('delta_t = {}'.format(delta_t))
            self.prev_clk = clk.nsecs

            target_linear_velocity = abs(self.twist_cmd.linear.x) # abs - fixing strange bug in twist_cmd ...
            target_angular_velocity = self.twist_cmd.angular.z
            current_linear_velocity = self.current_velocity.linear.x
            current_angular_velocity = self.current_velocity.angular.z
            current_yaw = helper.yaw_from_orientation(self.pose.pose.orientation)

            rospy.loginfo("tlv = {}, clv = {}, tav = {}, cav = {}, cyaw = {}".format(
                target_linear_velocity,
                current_linear_velocity,
                target_angular_velocity,
                current_angular_velocity,
                current_yaw))
            # rospy.loginfo('current_yaw = {}'.format(current_yaw))

            steer_cte = helper.calc_steer_cte(self.pose, self.final_waypoints)

            rospy.loginfo('STEER_CTE = {}'.format(steer_cte))

            throttle, brake, steering = self.controller.control(
                target_linear_velocity,
                current_linear_velocity,
                target_angular_velocity,
                current_angular_velocity,
                steer_cte,
                self.dbw_enabled) # <proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)

            # steering1 = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)

            rospy.loginfo("throttle, brake, steering = {}, {}, {}".format(throttle, brake, steering))
            # rospy.loginfo("steering1 = {}".format(steering1))

            if self.dbw_enabled:
              self.publish(throttle, brake, steering)
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
