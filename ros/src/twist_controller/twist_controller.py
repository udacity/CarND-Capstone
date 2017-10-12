import math
import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.pid_controller = PID(1.1, 0.01, 0.005, mn=kwargs['decel_limit'], mx=kwargs['accel_limit'])
        self.yaw_controller = YawController(kwargs['wheel_base'],
                                            kwargs['steer_ratio'],
                                            kwargs['min_speed'] + ONE_MPH,
                                            kwargs['max_lat_accel'],
                                            kwargs['max_steer_angle'])
        self.low_pass_filter = LowPassFilter(tau=0.5, ts=0.8)

        self.total_mass = kwargs['vehicle_mass'] + kwargs['fuel_capacity'] * GAS_DENSITY
        self.wheel_radius = kwargs['wheel_radius']
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']
        self.brake_deadband = kwargs['brake_deadband']

        self.last_run_time = rospy.get_time()

    def control(self, current_velocity, twist):
        now = rospy.get_time()
        sample_time = now - self.last_run_time

        target_linear_velocity = twist.linear.x
        target_angular_velocity = twist.angular.z

        filted_angular_velocity = self.low_pass_filter.filt(target_angular_velocity)
        steering = self.yaw_controller.get_steering(target_linear_velocity,
                                                   filted_angular_velocity,
                                                   current_velocity)

        linear_velocity_diff = target_linear_velocity - current_velocity
        accel = self.pid_controller.step(linear_velocity_diff, sample_time)

        if accel > 0:
            throttle = accel
            brake = 0.0
        else:
            throttle = 0.0
            desired_neg_acceleration = linear_velocity_diff / sample_time
            brake = -self.wheel_radius * self.total_mass * desired_neg_acceleration

        if math.fabs(accel) < self.brake_deadband:
            brake = 0.0

        self.last_run_time = now

        return throttle, brake, steering
