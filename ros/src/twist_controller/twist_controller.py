import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import time
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, accel_limit, decel_limit, brake_deadband,
                 max_lat_accel, max_steer_angle,
                 vehicle_mass, fuel_capacity, wheel_radius):
        # TODO: Implement
        self.steer_yaw = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.brake_deadband = brake_deadband
        self.accel = 0.0
        self.total_vehicle_mass = vehicle_mass + fuel_capacity*GAS_DENSITY
        self.wheel_radius = wheel_radius
        self.accel_pid = PID(0.6, 0.0, 0.0, decel_limit, accel_limit)
        self.accel_lpf = LowPassFilter(0.5, 0.5)
        self.prev_time = None

    def control(self, velocity, angular_velocity, target_velocity, target_angular_velocity):
        now = time.time()
        if self.prev_time:
            if target_velocity < 0.001:
                # If the target_velocity is a stop then apply the brake to hold position.
                # If the calculated accel is 0 due to 0 error the car will move.
                # Limit from https://discussions.udacity.com/t/what-is-the-range-for-the-brake-in-the-dbw-node/412339
                throttle = 0.0
                brake = -1809.0
            else:
                dt = now - self.prev_time

                velocity_error = target_velocity - velocity
                throttle, brake = self.acceleration(velocity_error, dt)
        else:
            throttle = 0.0
            brake = 0.0

        steer = self.steer_yaw.get_steering(target_velocity, target_angular_velocity, velocity)
        self.prev_time = now

        return throttle, abs(brake), steer

    def acceleration(self, velocity_error, dt):
        accel = self.accel_pid.step(velocity_error, dt)
        accel = self.accel_lpf.filt(accel)

        if accel <= 0.0 and accel > -self.brake_deadband:
            throttle = 0.0
            brake = 0.0

        elif accel > 0.0:
            # Acceleration required.
            throttle = min(accel, self.accel_limit)
            brake = 0.0

        else:
            # Braking required.  Brake values passed to publish should be in units of torque (N*m).
            # The correct values for brake can be computed using the desired acceleration,
            # weight of the vehicle, and wheel radius.
            # Note that https://discussions.udacity.com/t/what-is-the-range-for-the-brake-in-the-dbw-node/412339
            # suggests the `accel` value is limited by self.decel_limit rather than the valye of `brake`.
            throttle = 0.0
            brake = self.total_vehicle_mass * max(accel, self.decel_limit) * self.wheel_radius

        return throttle, brake

    def reset(self):
        self.prev_time = None
        self.accel_pid.reset()
        self.accel_lpf.reset()
        self.accel = 0.0
