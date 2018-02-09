
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from pid import PID
from yaw_controller import YawController
import math

class Controller(object):
    def __init__(self, *args, **kwargs):
        # PID-parameters (JS-XXX: change to dynamic reconfigure for tuning)
        steering_Kp = 1.0
        steering_Ki = 0.0
        steering_Kd = 0.0

        throttle_Kp = 1.0
        throttle_Ki = 0.0
        throttle_Kd = 0.0

        # create PID controllers for steering and throttle
        self.pid_steering = PID(steering_Kp, steering_Ki, steering_Kd)
        self.pid_throttle = PID(throttle_Kp, throttle_Ki, throttle_Kd,
                                kwargs['decel_limit'], kwargs['accel_limit'])

        # yaw-controller converts angular velocity to steering angles
        self.yaw_controller =  YawController(kwargs['wheel_base'],
                                             kwargs['steer_ratio'], 
                                             1,             # min-speed (JS-XXX check value)
                                             kwargs['max_lat_accel'], 
                                             kwargs['max_steer_angle'])

        # other variables
        self.dbw_enabled = False        # simulator starts with dbw disabled
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.wheel_radius = kwargs['wheel_radius']
        self.brake_deadband = kwargs['brake_deadband']

    def control(self, req_vel_linear, req_vel_angular, cur_vel_linear, cur_vel_angular, dbw_enabled):
        # reset PID controls after the safety driver enables drive-by-wire again
        #   JS-XXX: is this necessary or would it be beter to ignore this, the pid-controllers errors
        #           aren't updated will dbw is disabled...
        if not self.dbw_enabled and dbw_enabled:
            self.pid_steering.reset()
            self.pid_throttle.reset()

        self.dbw_enabled = dbw_enabled

        # early out when dbw is not enabled
        if not self.dbw_enabled:
            return 0.0, 0.0, 0.0

        # control steering
        steer = self.control_steering(req_vel_linear, req_vel_angular, cur_vel_linear, cur_vel_angular)

        # control velocity
        throttle, brake = self.control_velocity(req_vel_linear, cur_vel_linear)

        # return throttle, brake, steer
        return throttle, brake, steer

    def control_steering(self, req_vel_linear, req_vel_angular, cur_vel_linear, cur_vel_angular):  
        # update the error of the PID-controller
        pid_velocity = self.pid_steering.step(req_vel_angular - cur_vel_angular, 1)

        return self.yaw_controller.get_steering(cur_vel_linear, pid_velocity, cur_vel_linear)

    def control_velocity(self, req_vel_linear, cur_vel_linear):
        # update the error of the PID-controller
        value = self.pid_throttle.step(req_vel_linear - cur_vel_linear, 1)

        # engage throttle if the value of the PID-controller is positive
        if value > 0:
            throttle = value
            brake = 0.0
        # brake if the negative value of the PID-controller is outside of the brake-deadband
        elif math.fabs(value) > self.brake_deadband:
            throttle = 0
            brake = (self.vehicle_mass + (self.fuel_capacity * GAS_DENSITY)) * -value * self.wheel_radius

        return throttle, brake