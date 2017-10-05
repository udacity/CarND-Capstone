
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
CONTROL_FREQ = 50.0


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, vehicle_mass, wheel_radius, brake_deadband):
    	self.yaw_controller_ = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
    	self.steer_filter = LowPassFilter(0.5,1/CONTROL_FREQ)
    	self.brake_filter = LowPassFilter(0.5,1/CONTROL_FREQ)
    	self.throttle_filter = LowPassFilter(0.5,1/CONTROL_FREQ)
    	self.v_error_filter = LowPassFilter(0.05,1/CONTROL_FREQ)
    	self.brake_pid = PID(0.07,0,0,mn=0)
    	self.throttle_pid = PID(0.2,0,0,mn=0,mx=1)
    	self.vehicle_mass = vehicle_mass
    	self.brake_deadband = brake_deadband
    	self.wheel_radius = wheel_radius

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
    	steer = self.steer_filter.filt(self.yaw_controller_.get_steering(linear_velocity, angular_velocity, current_velocity))
    	
    	if not dbw_enabled:
    		self.brake_pid.reset()
    		self.throttle_pid.reset()
    		self.steer_filter = LowPassFilter(0.25,1/CONTROL_FREQ)
    		self.brake_filter = LowPassFilter(0.25,1/CONTROL_FREQ)
    		self.throttle_filter = LowPassFilter(0.25,1/CONTROL_FREQ)
    		self.v_error_filter = LowPassFilter(0.1,1/CONTROL_FREQ)
    		brake = 0
    		throttle = 0
    	else:
    		v_error = linear_velocity - current_velocity
    		#v_error = self.v_error_filter.filt(raw_v_error)
    		if v_error > 0:
    			#we need to accelerate
    			brake = 0
    			self.brake_filter = LowPassFilter(0.25,1/CONTROL_FREQ)
    			throttle = self.throttle_pid.step(v_error,1/CONTROL_FREQ)
    		else:
    			#use brake and not throttle
    			throttle = 0
    			self.throttle_filter = LowPassFilter(0.25,1/CONTROL_FREQ)
    			brake = self.brake_pid.step(-v_error, 1/CONTROL_FREQ)
    			brake = self.brake_deadband + brake*self.vehicle_mass*self.wheel_radius 

        # Return throttle, brake, steer
        return throttle, brake, steer
