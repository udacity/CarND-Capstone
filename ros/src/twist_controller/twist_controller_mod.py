import time

import rospy

from dynamic_reconfigure.server import Server
from pid import PID
from twist_controller.cfg import DynReconfConfig
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

    def __init__(self, default_update_interval, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, max_deceleration, max_acceleration, fuel_capacity, vehicle_mass, wheel_radius):
        self.fuel_capacity = fuel_capacity
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.current_timestep = None
        self.previous_acceleration = 0.
        self.max_deceleration = max_deceleration
        self.max_acceleration = max_acceleration
        self.default_update_interval = default_update_interval
        self.velocity_increase_limit_constant = 0.075
        self.velocity_decrease_limit_constant = 0.050
        self.braking_to_throttle_threshold_ratio = 4. / 3.

        self.yaw_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.dynamic_reconf_server = Server(
            DynReconfConfig, self.handle_dynamic_variable_update)

    def handle_dynamic_variable_update(self, config, level):
        # reset PID controller to use new parameters
        self.setup_pid_controllers(config['dyn_velo_proportional_control'], config[
            'dyn_velo_integral_control'], config['dyn_braking_proportional_control'], config['dyn_braking_integral_control'])

        return config

    def setup_pid_controllers(self, velo_p, velo_i, braking_p, braking_i):
        rospy.loginfo("Initializing PID controllers with velo_P: {}, velo_I: {}, braking_P: {}, braking_I: {}"
                      .format(velo_p, velo_i, braking_p, braking_i))
        # create velocity pid controller thresholded between min and max
        # acceleration values
        self.velocity_pid_controller = PID(
            velo_p, velo_i, 0, 0, 1)

        # create acceleration pid controller thresholded between 0% and 100%
        # for throttle
        self.braking_pid_controller = PID(
            braking_p, braking_i, 0.0, 0.0, 10000)

    def control(self, target_linear_velocity, target_angular_velocity, current_linear_velocity):
        # compute timestep
        timestep = self.compute_timestep()
        velocity_error = target_linear_velocity - current_linear_velocity

        if (target_linear_velocity == 0 and current_linear_velocity == 0):
            # reset integrators if we're at a stop
            self.reset()

        limit_constant = self.velocity_increase_limit_constant if velocity_error > 0 else self.velocity_decrease_limit_constant

        # use throttle if we want to speed up or if we want to slow down just
        # slightly
        throttle_command = self.velocity_pid_controller.step(
            velocity_error, timestep) if velocity_error > 0 or velocity_error > (-1 * limit_constant * current_linear_velocity) else 0

        # use brake if we want to slow down somewhat significantly or it looks
        # like want to stop
        brake_command = self.braking_pid_controller.step(-velocity_error, timestep) if velocity_error < (-1 * limit_constant *
                                                                                                         self.braking_to_throttle_threshold_ratio * current_linear_velocity) or (velocity_error < 0 and current_linear_velocity < 2.5) else 0

        rospy.logdebug('Current linear velocity %.2f, target linear velocity %.2f, throttle_command %.2f, brake_command %.2f',
                      current_linear_velocity, target_linear_velocity, throttle_command, brake_command)

        # Return throttle, brake, steer
        return (throttle_command,
                brake_command,
                self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity))

    def reset(self):
        self.last_timestep = None
        self.velocity_pid_controller.reset()
        self.braking_pid_controller.reset()

    def compute_timestep(self):
        last_timestep = self.current_timestep
        self.current_timestep = time.time()
        if last_timestep == None:
            last_timestep = self.current_timestep - self.default_update_interval
        return self.current_timestep - last_timestep
