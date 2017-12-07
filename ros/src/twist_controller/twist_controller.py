from pid import PID
from yaw_controller import YawController
import time

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
#   def __init__(self, *args, **kwargs):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.steer_yaw = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.prev_time = None

#   def control(self, *args, **kwargs):
    def control(self, velocity, angular_velocity, target_velocity, target_angular_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        if self.prev_time is None:
            throttle = 0.0
            brake = 0.0
            steer = 0.0
        else:
            throttle = 3.0
            brake = 0.0
            steer = self.steer_yaw.get_steering(target_velocity, target_angular_velocity, velocity)

        self.prev_time = time.time()

        return throttle, brake, steer
