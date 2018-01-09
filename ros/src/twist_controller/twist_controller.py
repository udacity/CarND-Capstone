import rospy
import math
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
from std_msgs.msg import Float32


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        '''
        Controller ROS node constructor
        '''

        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.min_speed = ONE_MPH

        self.yaw_controller = YawController(self.wheel_base,
                                            self.steer_ratio,
                                            self.min_speed,
                                            self.max_lat_accel,
                                            self.max_steer_angle)


    def control(self, *args, **kwargs):
        '''
        Compute throttle, brake and steer commands
        @param: args: target and current velocities
        @param: kwargs: empty
        @return: throttle, brake and steer commands
        '''

        target_linear_velocity_x = args[0]
        target_angular_velocity_z = args[1]
        current_linear_velocity_x = args[2]
        current_angular_velocity_z = args[3]

        # Compute the steering command
        steer = self.yaw_controller.get_steering(target_linear_velocity_x,
                                                 target_angular_velocity_z,
                                                 current_linear_velocity_x)

        accel = (target_linear_velocity_x - current_linear_velocity_x) / 0.5
        if accel > 0.0:
            accel = min(accel, self.accel_limit)
            throttle = accel / self.accel_limit
            brake = 0.0
        else:
            # accel <= 0.0 (deceleration)
            accel = max(accel, self.decel_limit)
            throttle = 0.0
            # force = mass * acceeration
            # torque = force * radius
            # brake torque = vehicle mass * acceleration * wheel radius
            brake = abs(self.vehicle_mass * accel * self.wheel_radius)

        return throttle, brake, steer
