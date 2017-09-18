from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import math

class Controller(object):

    def __init__(self, vehicle_mass, wheel_radius, accel_limit, decel_limit, 
                       wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # self.speed_controller = PID(5, 0.05, 1, -0.5, 0.5)

        # use a separate speed controller, than from PID
        self.speed_controller = SpeedController(
                                vehicle_mass,
                                wheel_radius,
                                accel_limit,
                                decel_limit)
        # self.speed_controller = PID(0.5, 0.02, 0.2)
        self.steering_controller = PID(5, 0.05, 1, -0.5, 0.5)
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

    def control(self, target_velocity, current_velocity, dbw_enabled, dt):
        target_linear_velocity, target_angular_velocity = target_velocity
        current_linear_velocity, current_angular_velocity = current_velocity

        # `rostopic echo /twist_cmd` says target linear velocity is fixed to 11.
        # but target angular velocity is changing based on vehicle's orienta
        linear_velocity_cte = target_linear_velocity - current_linear_velocity
        angular_velocity_cte = target_angular_velocity

        if not dbw_enabled: # manual driving
            self.speed_controller.reset()
            self.steering_controller.reset()

        corrective_steer = self.steering_controller.step(angular_velocity_cte, dt)
        predictive_steer = self.yaw_controller.get_steering(target_linear_velocity,
                                                            target_angular_velocity,
                                                            current_linear_velocity)
        steer = corrective_steer + predictive_steer

        throttle, brake = self.speed_controller.step(linear_velocity_cte, dt)

        # linear_velocity = self.speed_controller.step(linear_velocity_cte, dt)
        # throttle = 0
        # brake = 0

        # if linear_velocity > 0:
        #     throttle = linear_velocity
        # else:
        #     # brake = abs(linear_velocity) * 200000.
        #     acceleration = max(-5, linear_velocity_cte / dt)
        #     torque = 1736.35 * acceleration * 0.2413
        #     brake = min(abs(torque), 20000.)
        return throttle, brake, steer






class SpeedController(object):
    """ A Speed Controller class 
    Code modified from 
    https://github.com/kung-fu-panda-automotive/carla-driver/blob/master/ros/src/twist_controller/speed_controller.py
    """

    MAX_THROTTLE_TORQUE = 10.0
    MAX_BREAK_TORQUE = 20000.0

    def __init__(self, vehicle_mass, wheel_radius, accel_limit, decel_limit):
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit

    def step(self, error, sample_time):
        """
        """
        # calculate the acceleration based on the time
        # we need to make the change happen
        sample_time = 0.5
        acceleration = error / sample_time
        # apply limits to acceleration
        if acceleration > 0:
            acceleration = min(self.accel_limit, acceleration)
        else:
            acceleration = max(self.decel_limit, acceleration)
        # calculate torque = M*acc*R
        torque = self.vehicle_mass * acceleration * self.wheel_radius
        throttle, brake = 0, 0
        if torque > 0:
            # As documented, throttle is the percent of max torque applied
            throttle, brake = max(0.1, min(1.0, torque / SpeedController.MAX_THROTTLE_TORQUE)), 0.0
            # throttle, brake = min(1.0, torque), 0.0
        else:
            # brake is the torque we need to apply
            throttle, brake = 0.0, min(abs(torque), SpeedController.MAX_BREAK_TORQUE)

        return throttle, brake

    def reset(self):
        pass