"""
Author: Ravel Antunes <ravelantunes@gmail.com>
        Peng Xu <robotpengxu@gmail.com>
Date:   March 10, 2018
"""


import rospy
from throttle_controller import ThrottleController
from brake_controller import BrakeController
from yaw_controller import YawController
from pid import PID
import lowpass

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle,
                 vehicle_mass, wheel_radius, decel_limit, brake_deadband, fuel_capacity):

        # Initialize utility controllers
        self.throttle_control = ThrottleController()
        self.brake_control = BrakeController(vehicle_mass, wheel_radius, decel_limit, brake_deadband, fuel_capacity)
        self.yaw_control = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.throttle_pid = PID(kp=1.0, ki=1.0, kd=1.0)
        self.steering_pid = PID(kp=1.0, ki=1.0, kd=1.0)

        # Initialize state that will be updated nodes dbw is subscribed to
        self.velocity = None
        self.twist = None

        # init timestamp
        self.timestamp = rospy.get_time()

    def control(self, target_v, target_w, current_v, dbw_enabled):
        # Return 0 values if state not populated yet
        if not dbw_enabled:
            return 0.0, 0.0, 0.0

        rospy.loginfo('DBW enabled ...')

        dt = rospy.get_time() - self.timestamp

        error = target_v.x - current_v.x

        if error > 0:
            throttle, brake = self.throttle_control.control(error, dt)
            rospy.logdebug('throttle = %f, brake = %f' % (throttle, brake))
        else:
            throttle, brake = self.brake_control.control(error, dt)
            rospy.logdebug('throttle = %f, brake = %f' % (throttle, brake))

        steer = self.yaw_control.get_steering(target_v.x, target_w.z, current_v.x)

        return throttle, brake, steer
