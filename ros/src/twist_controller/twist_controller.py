from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import time

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
	self.low_pass_filter = LowPassFilter(0.8, 1.0)
	self.throttle_controller = PID(0.02, 0.01, 0.4)
	self.throttle_controller.reset()
	self.lastControlTime = None

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_status):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
	
	steering = self.yaw_controller.get_steering(linear_velocity, self.low_pass_filter.filt(angular_velocity), current_velocity)

	current_time = time.time()
	if self.lastControlTime is None:
		sample_time = 0.01
	else:
		sample_time = current_time - self.lastControlTime
	
	self.lastControlTime = current_time

	throttle = self.throttle_controller.step(linear_velocity-current_velocity, sample_time)

        return throttle, 0.0, steering
