#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane
import math
import copy
import tf
import numpy as np

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

        # TODO: Create `Controller` object
        self.controller = Controller(accel_limit=accel_limit, decel_limit=decel_limit,
                                     wheel_base=wheel_base, steer_ratio=steer_ratio, min_speed=1.0*0.447,
                                     max_lat_accel=max_lat_accel, max_steer_angle=max_steer_angle)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb)
        # Member vars
        self.dbw_enabled = True
        self.current_velocity = None
        self.twist_cmd = None
        self.cte_cnt = 0
        self.tot_cte = 0
        self.waypoints = None
        self.tf_listener = tf.TransformListener()
        self.loop()

    def waypoints_cb(self, msg):
        self.waypoints = msg

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
            if self.twist_cmd is None or self.current_velocity is None: # and self.waypoints !=None: 
                continue

            x = []
            y = []
            i = 0
            temp_waypoints = copy.deepcopy(self.waypoints)
            while len(x) < 20 and i < len(temp_waypoints.waypoints):
                # Transform waypoint to car coordinates
                temp_waypoints.waypoints[i].pose.header.frame_id = temp_waypoints.header.frame_id
                self.tf_listener.waitForTransform("/base_link", "/world", rospy.Time(0), rospy.Duration(10))
                transformed_waypoint = self.tf_listener.transformPose("/base_link", temp_waypoints.waypoints[i].pose)
                # Just add the x coordinate if the car did not pass the waypoint yet
                if transformed_waypoint.pose.position.x >= 0.0:
                    x.append(transformed_waypoint.pose.position.x)
                    y.append(transformed_waypoint.pose.position.y)
                i += 1
            coefficients = np.polyfit(x, y, 3)
            # We have to calculate the cte for a position ahead, due to delay
            cte = np.polyval(coefficients, 0.7 * self.current_velocity.twist.linear.x)
            cte *= abs(cte)
            rospy.loginfo('cte: %s', cte)
            self.tot_cte += abs(cte)
            self.cte_cnt += 1
            rospy.loginfo('avg_cte: %s', self.tot_cte / self.cte_cnt)
                
            throttle, brake, steering = self.controller.control(self.twist_cmd.twist.linear,
                self.twist_cmd.twist.angular,
                self.current_velocity.twist.linear,
                self.dbw_enabled,
                cte)

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

    def dbw_enabled_cb(self, msg):
        rospy.loginfo("DBW status changed to: %s", msg)
        self.dbw_enabled = msg

    def current_velocity_cb(self, msg):
        self.current_velocity = msg

    def twist_cmd_cb(self, msg):
        # rospy.loginfo("Received twist command %s", msg)
        self.twist_cmd = msg


if __name__ == '__main__':
    DBWNode()
