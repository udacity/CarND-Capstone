from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

# 1 gallon of gas weighs 6.3 pounds = 2.858 kg
GAS_DENSITY = 2.858
# 1 mile per hour = 0.44704 m/s
ONE_MPH = 0.44704

# Logging slows down everything down so bad it's lagging behind
# the simulator when logging to a visible console
DEBUG_LOG = False


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed=0.1,
                                            max_lat_accel=max_lat_accel, max_steer_angle=max_steer_angle)

        # Throttle PID controller parameters
        # Tuned using Ziegler-Nichols method, temporarily using mx=1.0
        # see https://en.wikipedia.org/wiki/PID_controller#Ziegler%E2%80%93Nichols_method
        # Which gives the parameters Ku=2.1, Tu=1.4s
        # Which in turn results in:
        kp = 1.26
        ki = 1.8
        kd = 0.294

        # Throttle range, 0 is minimum. 0.2 max for safety and comfort, real max is 1.0
        mn = 0.0
        mx = 0.2
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # Lowpass filter for measured velocity, the sensor is noisy
        tau = 0.5  # 1/(2*pi*tau) = cutoff frequency
        ts = 0.02  # Sample time at 50 Hz
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
            return 0.0, 0.0, 0.0

        if DEBUG_LOG:
            rospy.logwarn("Angular vel: {0}".format(angular_vel))
            rospy.logwarn("Target velocity: {0}".format(linear_vel))
            rospy.logwarn("Target angular velocity: {0}".format(angular_vel))
            rospy.logwarn("Current velocity: {0}".format(current_vel))
            rospy.logwarn("Filtered velocity: {0}".format(self.vel_lpf.get()))

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        # If we want to stop and are not going too fast, brake with known-good torque for staying still
        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0
            brake = 700  # Nm - according to Slack comments, minimum 550 Nm required, but use 700 to be safer

        # If no/low throttle was indicated and we want to slow down (decelerate), give zero throttle and brake
        elif throttle < 0.1 and vel_error < 0.0:
            throttle = 0.0
            decel = max(vel_error, self.decel_limit)  # decel_limit is negative as well
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius  # Braking torque in Nm

        # return 1., 0., 0.  # For debugging only, full gas ahead!
        return throttle, brake, steering
