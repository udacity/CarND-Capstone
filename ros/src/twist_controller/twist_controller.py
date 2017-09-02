import math
from collections import namedtuple
import rospy
from pid import PID
from yaw_controller import YawController

VehicleInfo = namedtuple('VehicleInfo',
                         ['mass', 'fuel_capacity', 'brake_deadband', 'decel_limit',
                          'accel_limit', 'wheel_radius', 'wheel_base', 'steer_ratio',
                          'max_lat_accel', 'max_steer_angle'])

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_info):
        self.pid_controller = PID(0.2, 0.002, 0.0, vehicle_info.decel_limit, vehicle_info.accel_limit)
        self.yaw_controller = YawController(vehicle_info.wheel_base, vehicle_info.steer_ratio,
                                            0, vehicle_info.max_lat_accel,
                                            vehicle_info.max_steer_angle)

        self.prev_time = None

    def control(self, twist_cmd, current_velocity, dbw_enabled):

        if not dbw_enabled:
            self.pid_controller.reset()
            return 0.0, 0.0, 0.0

        target_linear_velocity = twist_cmd.twist.linear.x
        target_angular_velocity = twist_cmd.twist.angular.z
        current_linear_velocity = current_velocity.twist.linear.x
        current_agular_velocity = current_velocity.twist.angular.z

        #rospy.loginfo("tl = %f, cl = %f, ta = %f", target_linear_velocity, current_linear_velocity, target_angular_velocity)

        linear_velocity_error = target_linear_velocity - current_linear_velocity
        time = rospy.get_time()
        delta_t = time - self.prev_time if self.prev_time is not None else 0
        self.prev_time = time
        linear_control = self.pid_controller.step(linear_velocity_error, delta_t)

        throttle = 0.0
        if linear_control > 0.0:
            throttle = linear_control

        brake = 0.0
        if linear_control < 0.0:
            brake = -linear_control

        steer = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)

        return throttle, brake, steer
