
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

USE_STEER_PID = True

from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import math
import rospy


class Controller(object):

    def __init__(self, params):
        self.yaw_controller = YawController(
            wheel_base = params['wheel_base'],
            steer_ratio = params['steer_ratio'],
            min_speed = params['min_speed'],
            max_lat_accel = params['max_lat_accel'],
            max_steer_angle = params['max_steer_angle'])

        self.prev_linear_velocity = 0.0

        self.filter_steer = True
        if self.filter_steer:
            self.steer_filter = LowPassFilter(time_interval=0.1,
                                              time_constant=1.0)

        self.filter_throttle = True
        if self.filter_throttle:
            self.throttle_filter = LowPassFilter(time_interval=0.1,
                                                 time_constant=0.1)

        self.break_constant = 0.1

        self.vehicle_mass = params['vehicle_mass']
        self.fuel_capacity = params['fuel_capacity']
        self.brake_deadband = params['brake_deadband']
        self.wheel_radius = params['wheel_radius']
        self.sample_rate = params['sample_rate']
        self.throttle_pid = PID(
            10.,
            .0,
            5.,
            0,
            1
        )
        self.steer_pid = PID(
            1.0,
            .0,
            5.0,
            -params['max_steer_angle'],
            params['max_steer_angle']
        )

        # assume tank is full when computing total mass of car
        self.total_mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY

    def control(self, linear_velocity, angular_velocity, current_velocity,
                enabled = True):

        velocity_diff = linear_velocity - current_velocity

        target_velocity_diff = linear_velocity - self.prev_linear_velocity

        time_interval = 1.0 / self.sample_rate

        throttle = 0.0
        brake = 0.0
        steer = 0.0
        if enabled:
            if target_velocity_diff < 0.0 or linear_velocity < 1.0:
                # Brake in torque [N*m]
                acc = velocity_diff/time_interval # Required acceleration
                brake = max(self.break_constant*math.fabs(acc), 0.19) * self.total_mass * self.wheel_radius
                self.throttle_pid.reset()
            else:
                throttle = self.throttle_pid.step(velocity_diff, time_interval)

                # Pass the low-pass filter
                if self.filter_throttle:
                    throttle = self.throttle_filter.filt(throttle)

            steer = self.yaw_controller.get_steering(
                linear_velocity,
                angular_velocity,
                current_velocity
            )
            if USE_STEER_PID:
                steer = self.steer_pid.step(angular_velocity, time_interval)
            else:
                steer = self.yaw_controller.get_steering(
                    linear_velocity,
                    angular_velocity,
                    current_velocity
                )
            # Pass the low-pass filter
            if self.filter_steer:
                steer = self.steer_filter.filt(steer)

            # Update the previous target
            self.prev_linear_velocity = linear_velocity

        else:
            self.throttle_pid.reset()
            self.steer_pid.reset()
            self.throttle_filter.reset()
            self.steer_filter.reset()

        # Logwarn for Debugging PID
        # run ```rosrun rqt_pt rqt_plot``` and set topics for plotting the actual velocity
        # rospy.logwarn("Throttle : {}".format(throttle))
        # rospy.logwarn("   Brake : {}".format(brake))
        # rospy.logwarn("   Steer : {}".format(steer))
        # rospy.logwarn("--- ")

        return throttle, brake, steer
