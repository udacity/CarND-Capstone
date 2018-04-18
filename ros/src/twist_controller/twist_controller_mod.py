import time

import rospy

from dynamic_reconfigure.server import Server
from lowpass import LowPassFilter
from pid import PID
from twist_controller.cfg import DynReconfConfig
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

    def __init__(self, default_update_interval, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, max_deceleration, max_throttle, fuel_capacity, vehicle_mass, wheel_radius):
        self.current_timestep = None
        self.previous_acceleration = 0.
        self.max_throttle = max_throttle
        self.default_update_interval = default_update_interval
        self.velocity_increase_limit_constant = 0.25
        self.velocity_decrease_limit_constant = 0.05
        self.braking_to_throttle_threshold_ratio = 4. / 3.
        self.manual_braking_upper_velocity_limit = 1.4
        self.braking_torque_to_stop = 100
        self.lpf_tau_throttle = 0.3
        self.lpf_tau_brake = 0.3
        self.lpf_tau_steering = 0.2

        self.max_braking_torque = (
            vehicle_mass + fuel_capacity * GAS_DENSITY) * abs(max_deceleration) * wheel_radius

        self.yaw_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.dynamic_reconf_server = Server(
            DynReconfConfig, self.handle_dynamic_variable_update)

        self.throttle_lpf = LowPassFilter(self.lpf_tau_throttle,
                                          default_update_interval)

        self.brake_lpf = LowPassFilter(self.lpf_tau_brake,
                                       default_update_interval)

        self.steering_lpf = LowPassFilter(
            self.lpf_tau_steering, default_update_interval)

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

    def control(self, target_linear_velocity, target_angular_velocity, current_linear_velocity, is_decelerating):
        # compute timestep
        timestep = self.compute_timestep()
        velocity_error = target_linear_velocity - current_linear_velocity

        if (target_linear_velocity == 0 and current_linear_velocity == 0):
            # reset integrators if we're at a stop
            self.reset()

        limit_constant = self.velocity_increase_limit_constant if velocity_error > 0 else self.velocity_decrease_limit_constant
        error_thresh = limit_constant * current_linear_velocity

        throttle_command = 0
        brake_command = 0
        control_mode = "Coasting"

        if is_decelerating and (target_linear_velocity < self.manual_braking_upper_velocity_limit and current_linear_velocity < self.manual_braking_upper_velocity_limit):
            # vehicle is coming to a stop or is at a stop; apply fixed braking torque
            # continuously, even if the vehicle is stopped
            brake_command = self.braking_torque_to_stop
            self.velocity_pid_controller.reset()
            control_mode = "Manual braking"
        elif velocity_error < -1 * limit_constant * current_linear_velocity:
            # use brake if we want to slow down somewhat significantly
            brake_command = self.braking_pid_controller.step(-velocity_error, timestep) if velocity_error < (-1 * limit_constant *
                                                                                                             self.braking_to_throttle_threshold_ratio * current_linear_velocity) or (velocity_error < 0 and current_linear_velocity < 2.5) else 0
            self.velocity_pid_controller.reset()
            control_mode = "PID braking"
        elif not is_decelerating or (current_linear_velocity > 5 and velocity_error > -1 * limit_constant * current_linear_velocity) or (current_linear_velocity < 5 and velocity_error > limit_constant * current_linear_velocity):
            # use throttle if we want to speed up or if we want to slow down
            # just slightly
            throttle_command = self.velocity_pid_controller.step(
                velocity_error, timestep)
            self.braking_pid_controller.reset()
            control_mode = "PID throttle"

        # apply low pass filter and maximum limit on brake command
        filtered_brake = min(
            self.max_braking_torque, self.brake_lpf.filt(brake_command))

        # do not apply throttle if any brake is applied
        if filtered_brake < 10:
            # brake is released, ok to apply throttle
            filtered_brake = 0
        else:
            # brake is still applied, don't apply throttle
            throttle_command = 0
            self.velocity_pid_controller.reset()

        # apply low pass filter and maximum limit on throttle command
        filtered_throttle = min(
            self.max_throttle, self.throttle_lpf.filt(throttle_command))

        rospy.loginfo('%s: current linear velocity %.2f, target linear velocity %.2f, is_decelerating %s, throttle_command %.2f, brake_command %.2f, error %.2f, thresh %.2f',
                      control_mode, current_linear_velocity, target_linear_velocity, is_decelerating, filtered_throttle, filtered_brake, velocity_error, error_thresh)

        # Return throttle, brake, steer
        return (filtered_throttle,
                filtered_brake,
                self.steering_lpf.filt(self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)))

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
