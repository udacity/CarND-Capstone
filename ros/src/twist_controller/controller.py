
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import math

class Controller(object):
    def __init__(self, *args, **kwargs):
        # create PID controller for throttle/brake
        #   the gains are set by the dynamic reconfigure server when starting the node using either
        #   the defaults in the PidGains.cfg file or parameters from the launch file
        self.pid_throttle = PID(1, 0, 0, kwargs['decel_limit'], kwargs['accel_limit'])

        # yaw-controller converts angular velocity to steering angles
        self.yaw_controller =  YawController(kwargs['wheel_base'],
                                             kwargs['steer_ratio'], 
                                             1,             # min-speed (JS-XXX check value)
                                             kwargs['max_lat_accel'], 
                                             kwargs['max_steer_angle'])

        # low-pass filter to smooth out steering values
        self.lp_filter = LowPassFilter(0.9, 0.8)

        # other variables
        self.dbw_enabled = False        # simulator starts with dbw disabled
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.wheel_radius = kwargs['wheel_radius']
        self.brake_deadband = kwargs['brake_deadband']
        self.last_t = None

    def update_throttle_gains(self, Kp, Ki, Kd):
        self.pid_throttle.set_gains(Kp, Ki, Kd)

    def control(self, cur_t, req_vel_linear, req_vel_angular, cur_vel_linear, cur_vel_angular, dbw_enabled):
        # reset PID controls after the safety driver enables drive-by-wire again
        #   JS-XXX: is this necessary or would it be beter to ignore this, the pid-controllers errors
        #           aren't updated will dbw is disabled...
        if not self.dbw_enabled and dbw_enabled:
            self.pid_throttle.reset()

        self.dbw_enabled = dbw_enabled

        # early out when dbw is not enabled
        if not self.dbw_enabled or self.last_t is None:
            self.last_t = cur_t
            return 0.0, 0.0, 0.0

        # time delta for sample
        delta_t = cur_t - self.last_t
        self.last_t = cur_t

        # control steering
        steer = self.control_steering(delta_t, req_vel_linear, req_vel_angular, cur_vel_linear, cur_vel_angular)

        # control velocity
        throttle, brake = self.control_velocity(delta_t, req_vel_linear, cur_vel_linear)

        # return throttle, brake, steer
        return throttle, brake, steer

    def control_steering(self, delta_t, req_vel_linear, req_vel_angular, cur_vel_linear, cur_vel_angular):  
        steer = self.yaw_controller.get_steering(req_vel_linear, req_vel_angular, cur_vel_linear)
        return self.lp_filter.filt(steer)

    def control_velocity(self, delta_t, req_vel_linear, cur_vel_linear):
        # update the error of the PID-controller
        value = self.pid_throttle.step(req_vel_linear - cur_vel_linear, delta_t)

        # engage throttle if the value of the PID-controller is positive
        if value > 0:
            throttle = value
            brake = 0.0
        # brake if the negative value of the PID-controller is outside of the brake-deadband
        elif math.fabs(value) > self.brake_deadband:
            throttle = 0.0
            brake = (self.vehicle_mass + (self.fuel_capacity * GAS_DENSITY)) * -value * self.wheel_radius
        else:
            throttle = 0.0
            brake = 0.0

        return throttle, brake