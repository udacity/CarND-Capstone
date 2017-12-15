#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
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

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Create `TwistController` object
        self.controller = Controller(steer_ratio, decel_limit, accel_limit, max_steer_angle, wheel_base, max_lat_accel)

        # Subscribe to all necessary topics
        rospy.Subscriber('/twist_cmd', TwistStamped, self.upd_twist)
        rospy.Subscriber('/current_velocity', TwistStamped, self.upd_velocity)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.upd_dbw_enabled)

        # Record data from subscribers
        self.twist_cmd = None
        self.current_velocity = None
        self.dbw_enabled = None

        # Logging data in csv file
        self.log_to_csv = True
        if self.log_to_csv:
            self.log_handle = self.log_init('dbw_node.csv')

        self.time_init = rospy.get_rostime()

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():

            if all([self.twist_cmd, self.current_velocity, self.dbw_enabled]):    # Ensure values have been initialized

                ########## for testing purposes only, do not integrate into master ####################
                #creates speed profile for testing longitudinal controller
                decel_setpoint = 1
                current_time = rospy.get_rostime()
                elapsed_time = current_time.secs - self.time_init.secs
                if  elapsed_time < 30:
                    speed_command = 13
                elif elapsed_time < 60:
                    speed_command = last_speed_command - .02*decel_setpoint    
                    speed_command = max(speed_command, 0)   
                elif elapsed_time < 90:
                    speed_command = 8   
                elif elapsed_time < 120:
                    speed_command = last_speed_command - .02*decel_setpoint    
                    speed_command = max(speed_command, 0) 
                else: 
                    speed_command = 11
                last_speed_command = speed_command
                ###################################################################################

                # Get predicted throttle, brake and steering
                throttle, brake, steering = self.controller.control(self.twist_cmd.linear.x,
                    self.twist_cmd.angular.z, self.current_velocity.linear.x, self.dbw_enabled, self.log_handle)

                # Log data for car control analysis
                if self.log_to_csv:
                    #timestamp = rospy.get_rostime() - self.time_init
                    self.log_data(rospy.get_rostime(), self.twist_cmd.linear.x, self.twist_cmd.angular.z,
                                  self.current_velocity.linear.x, self.current_velocity.angular.z, int(self.dbw_enabled), throttle, brake, steering)

                # Ensure dbw is enabled (not manual mode)
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

    def upd_twist(self, msg):
        self.twist_cmd = msg.twist
        lin, ang = self.twist_cmd.linear, self.twist_cmd.angular
        loginfo = 'twist_cmd x: {}, y: {}, z: {}, ang x: {}, y: {}, z: {}'.format(lin.x, lin.y, lin.z, ang.x, ang.y, ang.z)
        rospy.logdebug_throttle(1, loginfo)

    def upd_velocity(self, msg):
        self.current_velocity = msg.twist
        lin, ang = self.twist_cmd.linear, self.twist_cmd.angular
        loginfo = 'current_vel x: {}, y: {}, z: {}, ang x: {}, y: {}, z: {}'.format(lin.x, lin.y, lin.z, ang.x, ang.y, ang.z)
        rospy.logdebug_throttle(1, loginfo)

    def upd_dbw_enabled(self, msg):
        self.dbw_enabled = msg.data
        loginfo = 'dbw {}'.format(self.dbw_enabled)
        rospy.logdebug_throttle(1, loginfo)

    def log_init(self, log_path):
        log_handle = open(log_path,'w')
        headers = ','.join(["brkThrttle_p_effort", "brkThrttle_i_effort", "brkThrttle_d_effort", "steering_p_effort", "steering_i_effort", "steering_d_effort", "pid_throttle", "feedforward_throttle", "velocity_error", "DT", "decel_target", "latchBrake", "dbw_time", "target_linear_velocity", "target_angular_velocity","current_linear_velocity", "current_angular_velocity", "dbw_status", "throttle", "brake", "steering"])
        log_handle.write(headers + '\n')
        return log_handle
        
    def log_data(self, *args):
        self.log_handle.write(','.join(str(arg) for arg in args) + '\n')


if __name__ == '__main__':
    DBWNode()
