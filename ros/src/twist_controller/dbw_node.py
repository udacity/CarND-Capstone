#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane
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


        # publishers
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)
        # Contoller(kp, ki, kd)

        min_speed = 0
        self.controller = Controller(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # TODO: Subscribe to all the topics you need to

        # vehicle drive by wire
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        # vehicle current velocity
        rospy.Subscriber('/current_velocity', TwistStamped, self.crnt_vel_cb)
        # vehicle current twist
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        # vehicle position
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        # final waypoints
        rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb)


        rospy.logwarn("Checkpoint...... enetering loop...")

        self.time_last_sample = rospy.rostime.get_time()

        # enter drive loop
        self.loop()

    def dbw_enabled_cb(self, msg):
        # to check if drive by wire is enabled
        self.dbw_enabled_check = msg.data
        # toggles when the mode is changed in the simulator, e.g. checking the manuel checkbox
        # rospy.logwarn("dwb enabled: %s", msg.data) 

    def crnt_vel_cb(self, msg):
        # vehicle velocities:
        # self.vehicle_vel_lin = msg.twist.linear
        # self.vehicle_vel_ang = msg.twist.angular
        self.vehicle_cur_vel = self.magnitude(msg.twist.linear.x, msg.twist.linear.y)
        self.vehicle_cur_ang = msg.twist.angular.z
        
        # rospy.logwarn("current velocity: %s", msg.twist.linear.x)

    def twist_cmd_cb(self, msg):
        # vehicle twists:
        self.vehicle_vel_lin = msg.twist.linear
        self.vehicle_vel_ang = msg.twist.angular
        # self.vehicle_lin_vel
        # self.vehicle_ang_vel
        
        # rospy.logwarn("current twist: %s", msg.twist.linear.x)

    def pose_cb(self, msg):
        # TODO: Implement
        # get current pose of the vehicle
        # rospy.logwarn("current position of vehicle: %s", msg.pose.position.x)
        self.vehicle_pos = msg.pose.position

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def magnitude(self, x, y):
        return math.sqrt(x*x + y*y)
        
        

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        # rospy.logwarn("Entering loop now......")
        while not rospy.is_shutdown():
            time_elapsed = rospy.rostime.get_time() - self.time_last_sample
            self.time_last_sample = rospy.rostime.get_time()
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,

            # try to get the vehicle position, waypoints, next/nearest waypoint
            try:
                # pass
                # rospy.logwarn("Waypoint y: %s", self.waypoints.waypoints[0].pose.pose.position.y)
                # rospy.logwarn("Vehicle pose y: %s", self.vehicle_pos.y)
                cte = (self.waypoints.waypoints[0].pose.pose.position.y - self.vehicle_pos.y)
                # rospy.logwarn("Vehicle cte: %s", cte)

                throttle, brake, steering = self.controller.control(cte, 
                                        self.vehicle_vel_lin.x, self.vehicle_vel_ang.z, self.vehicle_cur_vel, self.vehicle_cur_ang, time_elapsed)

                rospy.logwarn("Vehicle steering: %s", steering)

                # rospy.logwarn("Vehicle current velocity: %s", self.vehicle_cur_vel)
                if not rospy.is_shutdown():

                    self.publish(throttle, brake, steering)
                    
                    # # check if dbw is enabled before publishing
                    # if self.dbw_enabled_check:
                    #     self.publish(throttle, brake, steering)
            except Exception as e:
                rospy.logwarn("Error: %s", e)
                pass
            

            

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
