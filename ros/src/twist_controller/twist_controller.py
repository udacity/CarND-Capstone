
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
            .105,
            .0,
            .15,
            0,
            1
        )
        self.steer_pid = PID(
            1.5,
            .0,
            .4,
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

        time_interval = 1.0 / self.sample_rate

        throttle = 0.0
        brake = 0.0
        steer = 0.0

        if enabled:
            if velocity_diff > 0:
                # put the velocity on sigmoid to smooth PID control
                logistic_target_velocity = 1 / (1 + math.exp(-1.9*(linear_velocity - 2.0)))
                throttle = self.throttle_pid.step(logistic_target_velocity, time_interval)
            elif velocity_diff < -0.5 or current_velocity < 2.0:
                brake = max(math.fabs(velocity_diff), 0.19) * self.total_mass * self.wheel_radius
                self.throttle_pid.reset()


            if USE_STEER_PID:
                steer = self.steer_pid.step(angular_velocity, time_interval)
            else:
                steer = self.yaw_controller.get_steering(
                    linear_velocity,
                    angular_velocity,
                    current_velocity
                )

        else:
            self.throttle_pid.reset()
            self.steer_pid.reset()

        return throttle, brake, steer
