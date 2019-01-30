import rospy

from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
LOGGING_THROTTLE_FACTOR = 3  # Only log after this many seconds
MAX_BRAKE = 400.0


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0  # Minimum throttle value
        mx = 0.4  # Maximum throttle value
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
        self.log_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0.0

        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0.0
            brake = MAX_BRAKE  # N*m - to hold the car in place if we are stopped at a light. Acceleration ~ 1m/s^2
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0.0
            decel = max(vel_error, self.decel_limit)
            brake = min(MAX_BRAKE, (abs(decel) * self.vehicle_mass * self.wheel_radius))  # Torque N*m

        if (current_time - self.log_time) > LOGGING_THROTTLE_FACTOR:
            self.log_time = current_time
            rospy.logwarn("POSE: current_vel={:.2f}, linear_vel={:.2f}, vel_error={:.2f}".format(current_vel,
                                                                                                 linear_vel,
                                                                                                 vel_error))
            rospy.logwarn("POSE: throttle={:.2f}, brake={:.2f}, steering={:.2f}".format(throttle, brake, steering))

        return throttle, brake, steering