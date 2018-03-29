from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(
        self, wheel_base, steer_ratio,
        min_speed, max_lat_accel, max_steer_angle
    ):
        # TODO: Implement
        self.throttle_controller = PID(0.03, 1000, 10, -5, 1)
        self.yaw_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel,
            max_steer_angle)

    def control(self, linear_velocity, angular_velocity, current_velocity):
        steer = self.yaw_controller.get_steering(
            linear_velocity,
            angular_velocity,
            current_velocity)
        activation = self.throttle_controller.step(
            linear_velocity - current_velocity, 0.02)
        if activation > 0:
            throttle, brake = activation, 0
        else:
            throttle, brake = 0, activation
        return throttle, brake, steer
