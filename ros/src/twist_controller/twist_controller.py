
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

USE_STEER_PID = True

from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import math


class Controller(object):

    def __init__(self, params):
        self.yaw_controller = YawController(
            wheel_base = params['wheel_base'],
            steer_ratio = params['steer_ratio'],
            min_speed = params['min_speed'],
            max_lat_accel = params['max_lat_accel'],
            max_steer_angle = params['max_steer_angle'])

        self.yaw_filter = LowPassFilter(0.5, 0.1)   
        self.filter_yaw = False

        self.vehicle_mass = params['vehicle_mass']
        self.fuel_capacity = params['fuel_capacity']
        self.brake_deadband = params['brake_deadband']
        self.wheel_radius = params['wheel_radius']
        self.sample_rate = params['sample_rate']
        self.throttle_pid = PID(
            1.0,
            .02,
            .2,
            0,
            1
        )
        self.steer_pid = PID(
            1.,
            .0,
            .6,
            -params['max_steer_angle'],
            params['max_steer_angle']
        )

        # assume tank is full when computing total mass of car
        self.total_mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY


    def control(self, linear_velocity, angular_velocity, current_velocity,
                enabled = True):

        if self.filter_yaw:
            angular_velocity = self.yaw_filter.filt(angular_velocity)

        velocity_diff = linear_velocity - current_velocity

        throttle = 0.0
        brake = 0.0
        steer = 0.0
        if enabled:
            if velocity_diff > 0:
                throttle = self.throttle_pid.step(velocity_diff, 1.0 / self.sample_rate)
            else:
                brake = -velocity_diff * self.total_mass * self.wheel_radius
                self.throttle_pid.reset()


            steer = self.yaw_controller.get_steering(
                linear_velocity,
                angular_velocity,
                current_velocity
            )
            if USE_STEER_PID:
                steer = self.steer_pid.step(steer, 1.0 / self.sample_rate)

        else:
            self.throttle_pid.reset()
            self.steer_pid.reset()

        return throttle, brake, steer
