from pid import PID
from yaw_control_PID import YawController
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

	

    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        min_speed = 0 

		self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        
        # from pid import as PID to calculate PID control
		kp=0.8
		ki=0
		kd=0.05
		mn=self.decel_limit
		mx=0.5
        self.throttle_control_PID = PID(kp, ki, kd, mn, mx * self.accel_limit)

        self.yaw_control_PID = YawController(self.wheel_base, self.steer_ratio, min_speed, self.max_lat_accel, self.max_steer_angle)

		kp_s=0.15
		ki_s=0.001
		kd_s=0.1
		mn_s=-self.max_steer_angle
		mx_s=self.max_steer_angle
        self.steering_control_PID = PID(kp_s, ki_s, kd_s, mn_s, mx_s)
        
    def reset(self):
        self.throttle_control_PID.reset()
        self.steering_control_PID.reset()
        
    def control(self, target_linear_velocity, target_angle_velocity, current_linear_velocity, CT_error, duration):
        
        # updating velocity links to throttle
        # calculate linear velocity error between proposed velocity and current velocity
        diff_velocity_error = target_linear_velocity - current_linear_velocity
        throttle = self.throttle_control_PID.step(diff_velocity_error, duration)
        brake = 0

        
        # methode is same as privious throttle calculation
        # calculate the steering value
        predictive_steering = self.yaw_control_PID.get_steering(target_linear_velocity, target_angle_velocity, current_linear_velocity)
        corrective_steering = self.steering_control_PID.step(CT_error, duration)
        steering = predictive_steering + corrective_steering


        # if velocity is less than 0 means need to decceleration
        # using throttle to calculate the brake value
        if(throttle < 0):
            deceleration = abs(throttle)
            brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * deceleration
            throttle = 0
        
        elif((target_linear_velocity==0.0) && (current_linear_velocity<0.1)):
            throttle = 0
            brake = 700  # torque required to hold the car in place when stopped at traffic light etc

        return throttle, brake, steering