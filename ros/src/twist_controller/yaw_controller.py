from math import atan
import constants as const

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        steering = 0.0
        radius = 0.0
        if abs(linear_velocity) >= const.NEAR_ZERO_FLOAT  and \
           abs(angular_velocity) >= const.NEAR_ZERO_FLOAT:
           radius = linear_velocity / angular_velocity
           angle = atan(self.wheel_base / radius) * self.steer_ratio
           steering = max(self.min_angle, min(self.max_angle, angle))
        return steering
