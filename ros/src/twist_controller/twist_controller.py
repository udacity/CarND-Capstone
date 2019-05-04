

from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.yaw_controller = YawController(..)
        self.throttle_pid = PID(kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM)
        pass

    def control(self, current_vel, dbw_enabled, target_linear_vel, target_angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        # vel_error = target_linear_vel - current_vel
        # throttle = self.throttle_pid.step(vel_error, sample_time)
        # brake =
        # steer = self.yaw_controller.get_steering(target_linear_vel, target_angular_vel, current_vel)
        return 1., 0., 0.
