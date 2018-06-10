import math
import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(
        self,
        vehicle_mass,
        fuel_capacity,
        brake_deadband,
        decel_limit,
        accel_limit,
        wheel_radius,
        wheel_base,
        steer_ratio,
        max_lat_accel,
        max_steer_angle):

        # Yaw controller to controller the turning
        vehicle_min_velocity = 0.1
        self.yaw_controller = YawController(
            wheel_base,
            steer_ratio,
            vehicle_min_velocity,
            max_lat_accel,
            max_steer_angle)

        kp, ki, kd, mn, mx = .3, .1, 0., 0., .2
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # Remove high frequency noise on the velocity
        tau, ts = .5, .02
        self.filter = LowPassFilter(tau, ts)

        # These can be used by a new controller to refine the driving
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(
        self,
        current_velocity,
        dbw_enabled,
        linear_velocity,
        angular_velocity):
        if not dbw_enabled:
            self.throttle_controller.reset()
            return .0, .0, .0

        current_velocity = self.filter.filt(current_velocity)

        steering = self.yaw_controller.get_steering(
            linear_velocity,
            angular_velocity,
            current_velocity)

        # Calculate the velocity error
        delta_velocity = linear_velocity - current_velocity
        self.last_velocity = current_velocity

        # Update time
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(delta_velocity, sample_time)
        brake = 0.

        eps = 1e-6
        if abs(linear_velocity) < eps and current_velocity < 0.1:
            # Keep car in place when it is stopped
            throttle = 0.
            brake = 400.

        elif throttle < .1 and abs(delta_velocity) < eps:
            throttle = 0.
            decel = max(delta_velocity, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        return throttle, brake, steering
