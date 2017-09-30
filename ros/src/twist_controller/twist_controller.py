# Credits: Wonderful SDC Slack Community

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        	
	self.vehicle_mass = args[0]
	self.fuel_capacity = args[1]
	self.brake_deadband = args[2]
	self.decel_limit = args[3]
	self.accel_limit = args[4]
	self.wheel_radius = args[5]
	self.wheel_base = args[6]
	self.steer_ratio = args[7]	
	self.max_lat_accel = args[8]
	self.max_steer_angle = args[9]

	self.min_speed = 0

	self.speed_controller = PID(0.5, 0.02, 0.2, 							self.decel_limit,self.accel_limit)
	self.steering_controller = PID(5, 0.05, 1, -0.5, 0.5)

	self.tau = 0.2
	self.ts = 0.1
	self.lpf = LowPassFilter(self.tau, self.ts)
	
	self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)

    def control(self, *args, **kwargs):
 
	target_linear_vel = args[0]
	target_angular_vel = args[1]
	current_linear_vel = args[2]
	current_angular_vel = args[3]
	dbw_en = args[4]
	dt = args[5]

	linear_vel_cte = (target_linear_vel - current_linear_vel)
	angular_vel_cte = target_angular_vel

	if dbw_en is False: #PID must not accumulate error under manual control
	    self.speed_controller.reset()
	    self.steering_controller.reset()

	linear_vel = self.speed_controller.step(linear_vel_cte, dt)
	linear_vel = self.lpf.filt(linear_vel)
	
	throttle = 0
	brake = 0
	
	if target_linear_vel < 0.001 and current_linear_vel < 0.447:
	    brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * 					self.wheel_radius * abs(self.decel_limit)
	    throttle = 0
	
	elif linear_vel > 0:	
	    throttle = linear_vel
	    brake = 0
	else:
	    
	    brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * 					self.wheel_radius * abs(linear_vel)
	    throttle = 0
  	
	corrective_steer = self.steering_controller.step(angular_vel_cte, dt)
	predictive_steer = self.yaw_controller.get_steering(target_linear_vel, 					target_angular_vel, current_linear_vel)
	steer = corrective_steer + predictive_steer

        return throttle, brake, corrective_steer
