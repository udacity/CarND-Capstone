#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32
import csv
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

        # Getting private namespace parameters
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
        min_car_speed = rospy.get_param('~min_car_speed', 0.01) # 1 cm/s, used in calc. yaw_rate.

        #Init variables
        self.current_velocity = 0.0
        self.current_angularv = 0.0
        self.goal_velocity = 0.0
        self.goal_angularv = 0.0
        self.dbw_enabled = False

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)
        self.custom_pub = rospy.Publisher('/vehicle/filt_vel',Float32,queue_size=1)
        self.custom_pub2 = rospy.Publisher('/vehicle/curr_vel', Float32, queue_size=1)

        self.start_time = rospy.get_time()

        # TODO: Create `TwistController` object
        self.controller = Controller(vehicle_mass,
                                     fuel_capacity,
                                     brake_deadband,
                                     decel_limit,
                                     accel_limit,
                                     wheel_radius,
                                     wheel_base,
                                     steer_ratio,
                                     max_lat_accel,
                                     max_steer_angle,
                                     min_car_speed)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/current_velocity', TwistStamped, self.curr_vel_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_cb)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # Get predicted throttle, brake, and steering using `twist_controller`
            # Should only publish the control commands if dbw is enabled
            throttle, brake, steering, filtered = self.controller.control(self.goal_velocity,
                                                                self.goal_angularv,
                                                                self.current_velocity,
                                                                self.current_angularv,
                                                                self.dbw_enabled)
            time_index = rospy.get_time() - self.start_time
            #if time_index < 50: throttle = 1.0
            #if time_index < 40: throttle = 0.6
            #if time_index < 30: throttle = 0.8
            #if time_index < 20: throttle = 0.1
            #throttle = 0.5
            #steering = 0.0

            if self.dbw_enabled:
                self.publish(throttle, brake, steering, filtered, float(self.current_velocity))
            rate.sleep()
            #with open('/tmp/utf.csv', 'ab') as fout:
            #    writer = csv.writer(fout)
                #writer.writerow([rospy.get_time(),self.current_velocity, self.goal_velocity, throttle])
            #    writer.writerow([time_index, throttle, self.current_velocity])

    def curr_vel_cb(self,msg):
        # TwistStamped structure from `rosmsg info geometry_msgs/TwistStamped` gives velocities:
        # x is direction of car, y is perpendicular to direction of travel. Right hand rule (RHR) assumed.
        self.current_velocity = msg.twist.linear.x
        self.current_angularv = msg.twist.angular.z



    def twist_cb(self,msg):
        # Same convention as before, same as used in course. x+ in direction of travel, RHR.
        self.goal_velocity = msg.twist.linear.x
        self.goal_angularv = msg.twist.angular.z

    def dbw_cb(self,msg):
        #rosmsg info std_msgs/Bool
        #bool data
        self.dbw_enabled = msg.data

    def publish(self, throttle, brake, steer, filtered, curr_vel):
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

        self.custom_pub.publish(filtered)
        self.custom_pub2.publish(curr_vel)


if __name__ == '__main__':
    DBWNode()
