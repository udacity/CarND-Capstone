#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math
from math import sqrt

from twist_controller import Controller
from styx_msgs.msg import Lane, Waypoint

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

        # TODO: Create `TwistController` object
        self.controller = Controller()

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        #rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)

        self.velocity = 0
        self.target_v = 0
        self.count = 0

        self.loop()

    def loop(self):
        rate = rospy.Rate(10) # 50Hz
        prev_throttle = 0
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            rospy.loginfo('dbw: target_v = ' + str(self.target_v))
            error = self.target_v - self.velocity
            throttle, brake, steering = self.controller.control(error)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)

            #if error < 0.1:
            #    throttle = 0
            
            brake = 0
            steering = 0
            self.publish(throttle, brake, steering)
            #self.publish(0.5)
            
            rate.sleep()

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        rospy.loginfo('dbw: waypoints_cb')

        waypoint = waypoints.waypoints[0]
        self.target_v = waypoint.twist.twist.linear.x 
        

        pass

    def current_velocity_cb(self, twist):
        # TODO: Implement
        rospy.loginfo('dbw: current_velocity_cb')
        x = twist.twist.linear.x
        y = twist.twist.linear.y
        z = twist.twist.linear.z
        self.velocity = x #sqrt(x**2 + y**2 + z**2)

        pass

    def twist_cmd_cb(self, twist):
        # TODO: Implement
        rospy.loginfo('dbw: twist_cmd_cb')
        x = twist.twist.linear.x
        y = twist.twist.linear.y
        z = twist.twist.linear.z
        self.velocity = x #sqrt(x**2 + y**2 + z**2)

        pass

    def current_pose_cb(self, pose):
        # TODO: Implement
        rospy.loginfo('dbw: current_pose_cb')
        x = twist.twist.linear.x
        y = twist.twist.linear.y
        z = twist.twist.linear.z
        self.velocity = sqrt(x**2 + y**2 + z**2)

        pass


    def publish(self, throttle=None, brake=None, steer=None):
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
        pass


if __name__ == '__main__':
    DBWNode()
