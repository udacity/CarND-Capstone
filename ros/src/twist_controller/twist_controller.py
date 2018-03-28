import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
GAS_DENSITY = 2.858
FULL_BRAKE_SPEED = 0.1  # If target velocity is smaller, apply full brake


class Controller(object):
    def __init__(self, vehicle_mass, decel_limit, accel_limit, wheel_radius,
                 wheel_base, steer_ratio, max_lat_accel, max_steer_angle,
                 brake_deadband, fuel_capacity, max_throttle_percent):
        self.acc_filter = LowPassFilter(4., 1.)
        self.velocity_pid = PID(1.5, 0.001, 0.,
                                mn=decel_limit, mx=max_throttle_percent)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 1,
                                            max_lat_accel, max_steer_angle)
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband
        self.total_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.decel_limit = decel_limit

    def reset(self):
        self.velocity_pid.reset()

    def control(self, twist, velocity, time_diff):
        velocity_cte = twist.twist.linear.x - velocity.twist.linear.x
        linear_acceleration = self.velocity_pid.step(velocity_cte, time_diff)
        steer = self.yaw_controller.get_steering(twist.twist.linear.x,
                                                 twist.twist.angular.z,
                                                 velocity.twist.linear.x)

        # Apply low-pass filter to the linear acceleration
        linear_acceleration = self.acc_filter.filt(linear_acceleration)
        # Keep full brake if target velocity is almost 0
        if twist.twist.linear.x < FULL_BRAKE_SPEED:
            throttle = 0.0
            brake_torque = self.acceleration_to_torque(abs(self.decel_limit))
        else:
            if linear_acceleration > 0.0:
                throttle = linear_acceleration
                brake_torque = 0.0
            else:
                throttle = 0.0
                deceleration = -linear_acceleration

                # Do not brake if too small deceleration
                if deceleration < self.brake_deadband:
                    deceleration = 0.0

                # Compute brake torque, in Nm
                brake_torque = self.acceleration_to_torque(deceleration)

        return throttle, brake_torque, steer

    def acceleration_to_torque(self, acceleration):
        """
        Transforms acceleration to torque
        Input: acceleration (float) acceleration, in m/s^2
        Output: torque (float) torque, in Nm
        """
        return acceleration * self.total_mass * self.wheel_radius