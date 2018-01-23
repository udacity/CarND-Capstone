import math
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        #pass
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        min_speed = 0
        self.linear_pid = PID(kp = 0.8, ki = 0, kd = 0.05, mn = self.decel_limit, mx = 0.5 * self.accel_limit)
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, min_speed, self.max_lat_accel, self.max_steer_angle)
        self.steering_pid = PID(kp = 0.15, ki = 0.001, kd = 0.1, mn = -self.max_steer_angle, mx = self.max_steer_angle)
    
    
    #def control(self, *args, **kwargs):
    def control(self, targ_lin_vel, targ_ang_vel, curr_lin_vel, cross_track_err, dura_secs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        #return 1., 0., 0.
        lin_vel_err = targ_lin_vel - curr_lin_vel
        
        vel_correct = self.linear_pid.step(lin_vel_err, dura_secs)
        
        brake = 0
        throttle = vel_correct
        
        if(throttle < 0):
            deceleration = abs(throttle)
            brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * deceleration if deceleration > self.brake_deadband else 0.
            throttle = 0
        
        predict_steering = self.yaw_controller.get_steering(targ_lin_vel, targ_ang_vel, curr_lin_vel)
        correct_steering = self.steering_pid.step(cross_track_err, dura_secs)
        steering = predict_steering + correct_steering
        
        return throttle, brake, steering
    
    
    def reset(self):
        self.linear_pid.reset()
        self.steering_pid.reset()