#!/usr/bin/env python

"""
This module implements TwistController
"""

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

import rospy


class TwistController:
    """This class implements a TwistController."""

    _vehicle_mass = None
    _fuel_capacity = None
    _brake_deadband = None
    _decel_limit = None
    _accel_limit = None
    _wheel_radius = None
    _wheel_base = None
    _steer_ratio = None
    _max_lat_accel = None
    _max_steer_angle = None

    _yaw_controller = None
    _throttle_controller = None

    _prev_time = None
    _prev_velocity = None

    def __init__(
        self,
        vehicle_mass,
        fuel_capacity,
        brake_deadband,
        decel_limit,
        accel_limit,
        wheel_radius,
        wheel_base,
        steer_ratio,
        max_lat_accel,
        max_steer_angle,
    ):

        self._vehicle_mass = vehicle_mass
        self._fuel_capacity = fuel_capacity
        self._brake_deadband = brake_deadband
        self._decel_limit = decel_limit
        self._accel_limit = accel_limit
        self._wheel_radius = wheel_radius
        self._wheel_base = wheel_base
        self._steer_ratio = steer_ratio
        self._max_lat_accel = max_lat_accel
        self._max_steer_angle = max_steer_angle

        self._yaw_controller = YawController(
            self._wheel_base, self._steer_ratio, 0.1, self._max_lat_accel, self._max_steer_angle
        )

        self._throttle_controller = PID(kp=0.3, ki=0.1, kd=0.0, mn=0.0, mx=0.2)

        self._vel_low_pass_filter = LowPassFilter(tau=0.5, ts=0.02)

        self._prev_time = rospy.get_time()

    def control(
        self,
        velocity,
        linear_velocity,
        angular_velocity,
        enabled,
    ):
        """Compute the next controlled values."""
        if not enabled:
            self._throttle_controller.reset()
            return (0.0, 0.0, 0.0)

        filtered_velocity = self._vel_low_pass_filter.filt(velocity)

        steering = self._yaw_controller.get_steering(
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            current_velocity=filtered_velocity,
        )
        vel_error = linear_velocity - filtered_velocity
        self._prev_velocity = filtered_velocity

        curr_time = rospy.get_time()
        sample_time = curr_time - self._prev_time
        self._prev_time = curr_time

        throttle = self._throttle_controller.step(vel_error, sample_time)

        brake = 0.0
        if abs(linear_velocity) < 1e-6 and filtered_velocity < 0.1:
            throttle = 0.0
            # N*m, to hold the car in place if we are stopped at a light.
            # Acceleration - 1m/s^2
            brake = 400

        elif throttle < 0.1 and vel_error < 0.0:
            throttle = 0.0
            decel = max(vel_error, self._decel_limit)
            brake = abs(decel) * self._vehicle_mass * self._wheel_radius  # Torque N*m

        return throttle, brake, steering
