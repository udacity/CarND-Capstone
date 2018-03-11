from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.steer = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

    def control(self, dt, cmd_x, cmd_theta, actual_x):
        steer = self.steer.get_steering(cmd_x, cmd_theta, actual_x)
        return 0.3, 0.0, steer
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        # return 1., 0., 0.
