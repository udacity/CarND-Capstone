import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # Yaw controller
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # PID controller
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0.  # Minimum throttle value
        mx = 0.2  # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5  # 1/(2pi*tau) = cutoff frequency
        ts = 0.02  # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        # control brake here
        # if we want to stop the car
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 700  # N*m to hold the car in place if we are stopped at a light. Acceleration ~ 1m/s^2

        # if we are going faster than we want
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius  # Torque N*m

        return throttle, brake, steering
