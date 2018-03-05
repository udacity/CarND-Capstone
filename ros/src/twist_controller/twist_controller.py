
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import rospy

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter


class YawControllerProperties:
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle


class PIDControllerProperties:
    def __init__(self, decel_limit, accel_limit):
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit


class LowPassFilterProperties:
    def __init__(self, wheel_radius, brake_deadband):
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband


class Controller(object):
    def __init__(self, pid_controller_properties, yaw_controller_properties, lowpass_filter_properties):

        # @todo Find suitable values for PID properties
        self.pid = PID(kp=0.1, ki=0.01, kd=0.001,
                       mn=pid_controller_properties.decel_limit, mx=pid_controller_properties.accel_limit)

        self.yaw_controller = YawController(yaw_controller_properties.wheel_base,
                                            yaw_controller_properties.steer_ratio,
                                            yaw_controller_properties.min_speed,
                                            yaw_controller_properties.max_lat_accel,
                                            yaw_controller_properties.max_steer_angle)

        # @todo Check what are actual correct properties for tau and ts for the lowpass filter
        self.low_pass_filter_pid = LowPassFilter(lowpass_filter_properties.brake_deadband, 1.0/50.0)
        self.low_pass_filter_yaw_controller = LowPassFilter(lowpass_filter_properties.wheel_radius, 1.0/50.0)


    def control(self, proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity, is_dbw_enabled):
        error = proposed_linear_velocity - current_linear_velocity
        rospy.loginfo("Gauss - Got error for PID: " + str(error))

        pid_result = self.pid.step(error=error, sample_time=1.0/50.0)
        rospy.loginfo("Gauss - PID Result: " + str(pid_result))

        if pid_result > 0.0:
            throttle = pid_result
            brake = 0.0
        else:
            throttle = 0.0
            brake = pid_result

        steer = self.yaw_controller.get_steering(proposed_linear_velocity,
                                                 proposed_angular_velocity,
                                                 current_linear_velocity)

        if not is_dbw_enabled:
            rospy.loginfo("Gauss - DBW not enabled, resetting PID, throttle, brake and steer")
            self.pid.reset()

            throttle = 0.0
            brake = 0.0
            steer = 0.0

        rospy.loginfo("Gauss - Throttle: " + str(throttle))
        rospy.loginfo("Gauss - Brake: " + str(brake))
        rospy.loginfo("Gauss - Steering: " + str(steer))

        # Return throttle, brake, steer
        return throttle, brake, steer
