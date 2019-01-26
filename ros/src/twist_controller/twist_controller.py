from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel,
                 max_steer_angle):

        # Create Yaw Controller to control steering
        self.yaw_controller = YawController(wheel_base, steer_ratio,
                                            0.1, max_lat_accel, max_steer_angle)

        # Setup PID controller to control throttle
        kp = 0.3
        ki = 0.1
        kd = 0
        mn = 0.
        mx = 0.2
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # Setup Low pass filter to filter out the noise in the velocity
        tau = 0.5
        ts = 0.02
        self.velocity_lpf = LowPassFilter(tau, ts)

        # Initialize variables
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, linear_velocity, angular_velocity,
                current_velocity, current_angular_velocity, dbw_enabled):

        # If DBW is not enabled, reset the PID controller to avoid accumulate errors
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        # filter the velocity noise
        current_velocity = self.velocity_lpf.filt(current_velocity)

        # Find out steering
        steering = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        # Find out throttle
        velocity_error = linear_velocity - current_velocity

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(velocity_error, sample_time)

        # Find out brake
        brake = 0
        if (linear_velocity == 0) and (current_velocity < 0.1):
            throttle = 0
            brake = 400
        elif (throttle < 0.1) and (velocity_error < 0):
            throttle = 0
            decel = max(velocity_error/sample_time, self.decel_limit)
            if abs(decel) > self.brake_deadband:
                brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        # Return throttle, brake, steering
        return throttle, brake, steering
