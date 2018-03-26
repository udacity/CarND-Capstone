from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        wheel_base 	     = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        min_velocity     = kwargs['min_velocity']
        max_lat_accel    = kwargs['max_lat_accel']
        max_steer_angle  = kwargs['max_steer_angle']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.brake_deadband = kwargs['brake_deadband']

        self.steer_pid = YawController(wheel_base, self.steer_ratio, min_velocity, max_lat_accel, max_steer_angle)
        self.vel_pid = PID(0.5, 0., 0., self.decel_limit, self.accel_limit)
        self.lowpassFilt = LowPassFilter(0.07, 0.02)

    def control(self, proposed_linear_vel, proposed_angular_vel, current_linear_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        #TODO: add safety at traffic lights, sharp turn, steer limits etc.
        # Return throttle, brake, steer
        # Acceleration Controller
        brake = 0.
        delta_throttle = self.vel_pid.step(proposed_linear_vel-current_linear_vel, 0.02)
        if delta_throttle > 0.:
            throttle = delta_throttle
            throttle = self.lowpassFilt.filt(throttle)
        elif delta_throttle < -self.brake_deadband:
            throttle = 0.
            brake = -delta_throttle
        else:
            throttle = 0.
        
        # Steering Controller
        if self.brake_deadband>0.1:
            if current_linear_vel > 0.05:
                steering = self.steer_pid.get_steering(proposed_linear_vel, proposed_angular_vel, current_linear_vel)
            else:
                steering = 0.
        else:
            steering = proposed_angular_vel * self.steer_ratio
        return throttle, brake, steering
    
    def reset(self):
        self.vel_pid.reset()
