
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

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
        self.velocity_pid = PID(
            1.0,
            0.1,
            0.4,
            mn=-math.fabs(params['decel_limit']),
            mx=math.fabs(params['accel_limit'])
        )
        #self.accel_pid = PID(
        #    5.0,
        #    0.1,
        #    0.5,
        #    mn=0.0,
        #    mx=1.0
        #)

    def control(self, linear_velocity, angular_velocity, current_velocity,
                enabled = True):

        if self.filter_yaw:
            angular_velocity = self.yaw_filter.filt(angular_velocity)

        velocity_diff = linear_velocity - current_velocity
        needed_accel = self.velocity_pid.step(velocity_diff, 1.0 / self.sample_rate)

        if needed_accel > 0:
            throttle = needed_accel
        else:
            throttle = 0.0
            self.velocity_pid.reset()

        # assume tank is full
        total_mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY

        if (needed_accel < -math.fabs(self.brake_deadband)):
            brake = -needed_accel * total_mass * self.wheel_radius
        else:
            brake = 0.0

        steer = self.yaw_controller.get_steering(
            linear_velocity,
            angular_velocity,
            current_velocity)

        return throttle, brake, steer
