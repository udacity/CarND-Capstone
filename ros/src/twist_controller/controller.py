
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import math

# Note: how to tune the gains of the PID controllers at runtime
# - the gains can be changed at runtime using the ROS dynamic_reconfigure package
# - while the ROS session is running : 
#     - start rqt and add a Dynamic Reconfigure plugin (Plugins > Configuration)
#     - or use the included perspective file : rqt --perspective-file src/twist_controller/launch/dbw_node.perspective
# - select 'dbw_node' in the list to view and change the PID-gains
# - to make changes permanent: modify PidGains.cfg in the src/twist_controller/cfg directory or add a value to the parameter list in the launch file
#     - don't forget to run catkin_make after changing PidGains.cfg
#     - if necessary this could enables us to use different gains on Carla (dbw.launch) than in the simulator (dbw_sim.launch)

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
            brake = self.compute_brake(value)
        else:
            throttle = 0.0
            brake = 0.0

        # apply the brakes a bit when the car is supposed to be standing still
        # (to avoid the car creeping forward due to the small jitter of the PID-controller)
        # note: doesn't override larger values coming from the PID-controller
        if req_vel_linear == 0 and cur_vel_linear < 0.5:
            throttle = 0.0
            brake = max(brake, self.compute_brake(-1.0))

        return throttle, brake

    def compute_brake(self, pid_value):
        return (self.vehicle_mass + (self.fuel_capacity * GAS_DENSITY)) * -pid_value * self.wheel_radius

