#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

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

        # TODO: Create `TwistController` object
        self.controller = Controller(kp=0.8, ki=0.0, kd=0.6, max_accel= accel_limit, max_decel= decel_limit)
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # TODO: Subscribe to all the topics you need to

        self.dbw_enabled = False
        self.controller_reset = False
        self.current_velocity = 0.0
        self.ref_velocity = 0.0
        self.ref_angular_velocity = 0.0

        self.prev_time = rospy.Time(0).to_sec()

        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.set_dbw_enabled)

        rospy.Subscriber('/current_velocity', TwistStamped, self.set_current_velocity)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.set_twist_cmd)
        
        self.loop()


    def set_dbw_enabled(self, dbw_enabled):
        self.dbw_enabled = dbw_enabled

    def set_current_velocity(self, velocity):
        self.current_velocity = velocity.twist.linear.x

    def set_twist_cmd(self, twist):
        # self.ref_velocity = twist.twist.linear.x
        self.ref_velocity = abs(twist.twist.linear.x) # temporary solution to negative velocities
        self.ref_angular_velocity = twist.twist.angular.z
        

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        current_time = rospy.Time.now().to_sec()
        sample_time = current_time - self.prev_time #float(1.0/50.0) # 50Hz????????????????? 
        self.prev_time = current_time
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            rospy.loginfo("""DBW enabled: {}""".format(self.dbw_enabled))
            # rospy.loginfo("""Ref Velocity: {} - Ref Angular V.: {} - Curr V.: {}""".format(self.ref_velocity, self.ref_angular_velocity, self.current_velocity))
            if self.dbw_enabled:
                if not self.controller_reset:
                    self.controller.reset()
                    self.controller_reset = True
                # steer = self.yaw_controller.get_steering(self.ref_velocity, self.ref_angular_velocity, self.current_velocity)
                
                steer = self.yaw_controller.get_steering(self.current_velocity, self.ref_angular_velocity, self.current_velocity)
                velocity_error = self.ref_velocity - self.current_velocity
                rospy.loginfo("""Velocity Ref: {} - Curr: {} - Err: {}""".format(self.ref_velocity, self.current_velocity, velocity_error))
                throttle, brake = self.controller.control(velocity_error, sample_time)
                self.publish(throttle, brake, steer)
            else:
                self.controller_reset = False
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
