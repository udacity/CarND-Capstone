import time

import rospy

from dynamic_reconfigure.server import Server
from pid import PID
from twist_controller.cfg import DynReconfConfig
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, max_deceleration, max_acceleration, fuel_capacity, vehicle_mass, wheel_radius):
        self.fuel_capacity = fuel_capacity
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.current_timestep = None
        self.previous_acceleration = 0.
        self.max_deceleration = max_deceleration
        self.max_acceleration = max_acceleration

        self.yaw_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.dynamic_reconf_server = Server(
            DynReconfConfig, self.handle_dynamic_variable_update)

    def handle_dynamic_variable_update(self, config, level):
        # reset PID controller to use new parameters
        self.setup_pid_controller(config['dyn_velo_proportional_control'], config[
                                  'dyn_velo_integral_control'], config['dyn_velo_differential_control'])

        return config

    def setup_pid_controller(self, p, i, d):
        rospy.loginfo("Initializing PID controller with P: {}, I: {}, D: {}"
                      .format(p, i, d))
        self.velocity_pid_controller = PID(
            p, i, d, self.max_deceleration, self.max_acceleration)

    def control(self, target_linear_velocity, target_angular_velocity, current_linear_velocity):
        # use velocity pid controller to compute new velocity
        acceleration = self.compute_acceleration(
            target_linear_velocity - current_linear_velocity)
        rospy.logdebug('Current linear velocity %s, target linear velocity %s, computed acceleration %s',
                       current_linear_velocity, target_linear_velocity, acceleration)

        # Return throttle, brake, steer
        return (acceleration if acceleration > 0. else 0.,
                self.compute_brake_torque(abs(acceleration)) if acceleration < 0. else 0.,
                self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity))

    def reset(self):
        self.last_timestep = None
        self.velocity_pid_controller.reset()

    def compute_acceleration(self, error):
        last_timestep = self.current_timestep
        self.current_timestep = int(round(time.time() * 1000))

        if (last_timestep == self.current_timestep):
            rospy.loginfo(
                'No timestep advance since last acceleration computation; returning previous acceleration')
            return self.previous_acceleration

        # TODO - modify PID instance to modify error value to prevent sudden control jumps when re-engaging DBW
        # after a period of inactivity
        if last_timestep is None:
            # attempted to compute values for first time; do not accellerate
            return 0.

        self.previous_acceleration = self.velocity_pid_controller.step(
            error, self.current_timestep - last_timestep)
        return self.previous_acceleration

    def compute_brake_torque(self, deceleration):
        # deceleration in m/s^2 needs to be converted to torque in Nm.
        return (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * deceleration * self.wheel_radius
