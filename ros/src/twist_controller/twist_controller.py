from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import time

GAS_DENSITY = 2.858
ONE_MPH = 0.44704               # 1 miles/hour in meters/second
MAX_SPEED_metersps = 35.0 * ONE_MPH  # MPH in meters/second

class TwistController(object):
    def __init__(self, wheel_base, vehicle_mass, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
    #def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.throttle_controller = PID(2.7, 0.01, 0.02, mn=0.0, mx=1.0)
        self.yaw_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.lowpass_filter = LowPassFilter(0.7, 1.0)
        self.brake_coefficient = 10.0  # tentative guess
        self.vehicle_mass = vehicle_mass
        self.prev_time = None

    def control(self, desired_linear_velocity, desired_angular_velocity, current_linear_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if self.prev_time is None:
            self.prev_time = time.time()
            return 0., 0., 0.

        smoothed_current_linear_velocity = self.lowpass_filter.filt(desired_angular_velocity)
        desired_linear_velocity_modulated = min(
            MAX_SPEED_metersps, smoothed_current_linear_velocity)

        error_linear_velocity = (desired_linear_velocity_modulated -
                                 current_linear_velocity)
        if error_linear_velocity < 0: # need to deceleration
            brake = self.brake(error_linear_velocity, current_linear_velocity, self.vehicle_mass)
            throttle = 0
        else:
            elapsed_time = time.time() - self.prev_time
            throttle = self.throttle_controller.step(error_linear_velocity, elapsed_time)
            throttle = max(min(throttle, 1.0), 0.0)
            brake = 0.
        # end of if error_linear_velocity < 0

        steer = self.yaw_controller.get_steering(
            desired_linear_velocity, desired_angular_velocity, smoothed_current_linear_velocity)
        self.prev_time = time.time()
        return throttle, brake, steer

    def brake(self, error_in_linear_velocity, current_linear_velocity, vehicle_mass):
        # might be more fine tuned, might consider vehicle's mass, and the current velocity, etc.
        # might use another PID.
        brake_v = -self.brake_coefficient * error_in_linear_velocity  # assume the brake_v should be positive
        return max(brake_v, 1.0)
