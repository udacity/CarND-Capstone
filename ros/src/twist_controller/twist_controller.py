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
        self.throttle_controller = PID(1, 1, 1)
        self.yaw_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel,
            max_steer_angle)
    
    def control(self, linear_velocity, angular_velocity, current_velocity):
        steer = self.yaw_controller.get_steering(
            linear_velocity, angular_velocity, current_velocity)
        # Return throttle, brake, steer
        return 1., 0., steer
