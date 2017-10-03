
from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
    	self.yaw_controller_ = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
    	self.brake_pid = PID(1,0,0)
    	self.throttle_pid = PID(1,0,0)

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
    	steer = self.yaw_controller_.get_steering(linear_velocity, angular_velocity, current_velocity)
    	
    	if not dbw_enabled:
    		self.brake_pid.reset()
    		self.throttle_pid.reset()
    		brake = 0
    		throttle = 0
    	elif linear_velocity > current_velocity:
    		#use throttle and not brake
    		brake = 0
    		throttle = self.throttle_pid.step(linear_velocity - current_velocity,0.02)
    	else:
    		#use brake and not throttle
    		throttle = 0
    		brake = self.brake_pid.step(linear_velocity - current_velocity, 0.02)

        # Return throttle, brake, steer
        return throttle, brake, steer
