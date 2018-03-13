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

        self.yaw_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.dynamic_reconf_server = Server(
            DynReconfConfig, self.handle_dynamic_variable_update)

    def handle_dynamic_variable_update(self, config, level):
        # reset PID controller to use new parameters
        self.setup_pid_controllers(config['dyn_velo_proportional_control'], config[
            'dyn_accel_proportional_control'], config['dyn_accel_integral_control'], self.max_deceleration, self.max_acceleration)

        # reset acceleration cruise constant
        self.acceleration_cruise_constant = config['dyn_acceleration_cruise_constant']

        return config

    def setup_pid_controllers(self, velo_p, accel_p, accel_i, max_deceleration, max_acceleration):
        rospy.loginfo("Initializing PID controllers with velo_P: {}, accel_P: {}, accel_I: {}"
                      .format(velo_p, accel_p, accel_i))
        # create velocity pid controller thresholded between min and max
        # acceleration values
        self.velocity_pid_controller = PID(
            velo_p, 0, 0, max_deceleration, max_acceleration)

        # create acceleration pid controller thresholded between 0% and 100%
        # for throttle
        self.acceleration_pid_controller = PID(accel_p, accel_i, 0.0, 0.0, 1.0)

    def control(self, target_linear_velocity, target_angular_velocity, current_linear_velocity):
        # compute timestep
        timestep = self.compute_timestep()
        # use velocity pid controller to compute desired acceleration
        acceleration_command = self.compute_acceleration_command(
            target_linear_velocity - current_linear_velocity, timestep)
        rospy.logwarn('Current linear velocity %.2f, target linear velocity %.2f, desired total acceleration %.2f',
                      current_linear_velocity, target_linear_velocity, acceleration_command)

        # Return throttle, brake, steer
        # TODO - build "coasting" to reduce speed rather than always using positive throttle or brake
        return (self.compute_throttle(acceleration_command, target_linear_velocity, timestep) if acceleration_command > 0. else 0.,
                self.compute_brake_torque(abs(acceleration_command)) if acceleration_command < 0. else 0.,
                self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity))

    def reset(self):
        self.last_timestep = None
        self.velocity_pid_controller.reset()
        self.acceleration_pid_controller.reset()

    def compute_timestep(self):
        last_timestep = self.current_timestep
        self.current_timestep = time.time()
        if last_timestep == None:
            last_timestep = self.current_timestep - self.default_update_interval
        return self.current_timestep - last_timestep

    def compute_acceleration_command(self, velocity_error, timestep):
        return self.velocity_pid_controller.step(velocity_error, timestep)

    def compute_throttle(self, acceleration, target_linear_velocity, timestep):
        # compute throttle based on desired acceleration plus a small amount of
        # additional acceleration to combat wind resistance
        return self.acceleration_pid_controller.step(acceleration + target_linear_velocity * self.acceleration_cruise_constant, timestep)

    def compute_brake_torque(self, deceleration_command):
        # deceleration in m/s^2 needs to be converted to torque in Nm.
        return (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * deceleration_command * self.wheel_radius
