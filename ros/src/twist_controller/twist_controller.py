
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
CONTROL_FREQ = 50.0


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, vehicle_mass, wheel_radius, brake_deadband):
    	self.yaw_controller_ = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
    	self.steer_filter = LowPassFilter(0.25,1/CONTROL_FREQ)
    	self.brake_filter = LowPassFilter(0.25,1/CONTROL_FREQ)
    	self.throttle_filter = LowPassFilter(0.25,1/CONTROL_FREQ)
    	self.brake_pid = PID(50,0,0)
    	self.throttle_pid = PID(10,0,0)
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
    		brake = 0
    		throttle = 0
    	elif linear_velocity > current_velocity:
    		#use throttle and not brake
    		brake = 0
    		self.brake_filter = LowPassFilter(0.25,1/CONTROL_FREQ)
    		raw_throttle = self.throttle_pid.step(linear_velocity - current_velocity,1/CONTROL_FREQ)
    		throttle = self.throttle_filter.filt(raw_throttle)
    	else:
    		#use brake and not throttle
    		throttle = 0
    		self.throttle_filter = LowPassFilter(0.25,1/CONTROL_FREQ)
    		raw_brake = self.brake_pid.step(linear_velocity - current_velocity, 1/CONTROL_FREQ)
    		brake = self.brake_filter.filt(raw_brake)
    		brake = -1*(self.brake_deadband + brake*self.vehicle_mass*self.wheel_radius) 

        # Return throttle, brake, steer
        return throttle, brake, steer
