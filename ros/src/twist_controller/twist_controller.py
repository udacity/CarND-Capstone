import rospy
from lowpass import LowPassFilter
from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


#from pid import PID
#from lowpass import LowPassFilter
#from yaw_controller import YawController
#import rospy

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
	self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
	
	kp = 0.3
	ki = 0.1
	kd = 0.0
	min_throttle = 0.0
	max_throttle = 0.5
	self.throttle_controller = PID(kp, ki, kd, min_throttle, max_throttle)
	
	tau = 0.01#0.5
	ts = 0.5#0.02
	self.vel_lpf = LowPassFilter(tau, ts)
	
	self.vehicle_mass = vehicle_mass
	self.fuel_capacity = fuel_capacity
	self.brake_deadband = brake_deadband
	self.decel_limit = decel_limit
	self.accel_limit = accel_limit
	self.wheel_radius = wheel_radius
	
	self.last_time = rospy.get_time()
	


    def control(self, linear_vel, angular_vel, current_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
        	self.throttle_controller.reset()
        	return 0.0, 0.0, 0.0
        
        current_vel = self.vel_lpf.filt(current_vel)
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        #rospy.logwarn("[ LOG ] : linear_vel = %s    angular_vel = %s    current_vel = %s/    steering = %sn", linear_vel, angular_vel, current_vel, steering)
        #rospy.logwarn("proposed_speed = %s", linear_vel)
        
     	vel_error = linear_vel - current_vel
     	
     	self.last_vel = current_vel
     	
     	current_time = rospy.get_time()
     	sample_time = current_time - self.last_time
     	self.last_time = current_time
     	
     	throttle = self.throttle_controller.step(vel_error, sample_time)
     	brake = 0
     	
     	#if linear_vel == 0.0 and current_vel < 0.1:
     	#	throttle = 0
     	#	brake = 700 # value  of torque in N*m  
     	#elif throttle < 0.1 and vel_error < 0:
     	#	throttle = 0
     	#	decel = max(vel_error, self.decel_limit)
     	#	brake = abs(decel) * self.vehicle_mass * self.wheel_radius
        
        # Measures against small throttle while breaking (real car would not like that, strain on the brakes)
        if (throttle < 0.00001):
            brake = 100
            throttle = 0
        
        # Disengage brake when accelerating
        if (vel_error > 0):
            brake = 0
        
        # If deceleration intended use adaptive brake taking into account current speed / intended speed difference
        if (-vel_error > 0):
            if (current_vel > 0.5):
                if (linear_vel < current_vel / 1.5):
                    brake = 100 * (((current_vel / (linear_vel + 0.000001))) * 5.0)
                    if (brake > 4000):
                        brake = 4000
            else:
                brake = 100
        
        if (brake > 0):
            throttle = 0
        
        return throttle, brake, steering

