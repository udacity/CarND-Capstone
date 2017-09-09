import math

import pid

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_LINEAR_VELOCITY = 0.0
MAX_LINEAR_VELOCITY = 1.0
MIN_ANGULAR_VELOCITY = -0.5
MAX_ANGULAR_VELOCITY = 0.5
THROTTLE_P_VAL = 1.0
THROTTLE_I_VAL = 0.1
THROTTLE_D_VAL = 0.1
STEER_P_VAL = 1.0
STEER_I_VAL = 0.1
STEER_D_VAL = 0.1


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.throttle_pid = pid.PID(THROTTLE_P_VAL, THROTTLE_I_VAL, THROTTLE_D_VAL, mn=MIN_LINEAR_VELOCITY, mx=MAX_LINEAR_VELOCITY)
        self.steering_pid = pid.PID(STEER_P_VAL, STEER_I_VAL, STEER_D_VAL, mn=MIN_ANGULAR_VELOCITY, mx=MAX_ANGULAR_VELOCITY)

    def control(self, target_linear_velocity, target_angular_velocity, 
                current_linear_velocity, current_angular_velocity,
                dbw_status, sample_time, **kwargs):

        throttle_error = target_linear_velocity - current_linear_velocity
        steering_error = target_angular_velocity - current_angular_velocity
        
        throttle = self.throttle_pid.step(throttle_error, sample_time)
        brake = 0.
        steering = self.steering_pid.step(steering_error, sample_time)
        
        # Return throttle, brake, steering
        return throttle, brake, steering
