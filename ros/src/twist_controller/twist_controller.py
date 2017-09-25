
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from yaw_controller import YawController
from lowpass import LowPassFilter


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

    def control(self, linear_velocity, angular_velocity, current_velocity,
                enabled = True):

        if self.filter_yaw:
            angular_velocity = self.yaw_filter.filt(angular_velocity)

        throttle = 1.0
        brake = 0.0
        steer = self.yaw_controller.get_steering(
            linear_velocity,
            angular_velocity,
            current_velocity)

        return throttle, brake, steer
