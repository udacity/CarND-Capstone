import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Controller Class
class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,
                 decel_limit, accel_limit, wheel_radius, wheel_base,
                 steer_ratio, max_lat_accel, max_steer_angle):

        # Initialize yaw controller
        self.yaw_controller = YawController(
            wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # PID controller for throttle
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0. # Minimum throttle value
        mx = 0.2 # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = .02 # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        """Gets predicted throttle, brake, and steering

        Args:
            current_vel: current vehicle velocity
            dbw_enabled: checks if dbw is nabled or not
            linear_vel: desired linear velocity
            angular_velocity: desired angular velocity

        Returns:
            throttle
            brake
            steering
        """

        # if dbw is not enabled
        if not dbw_enabled:
            # reset throttle controller (to avoid accumulate error)
            self.throttle_controller.reset()
            return 0., 0., 0.

        # Filter noisy current velocity with low pass filter
        current_vel = self.vel_lpf.filt(current_vel)

        # rospy.logwarn("Angular vel: {0}".format(angular_vel))
        # rospy.logwarn("Target velocity: {0}".format(linear_vel))
        # rospy.logwarn("Target angular velocity: {0}".format(angular_vel))
        # rospy.logwarn("Current velocity: {0}".format(current_vel))
        # rospy.logwarn("Filtered velocity: {0}".format(self.vel_lpf.get()))

        # Get steering from yaw controller
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        # Calculate velocity error
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        # Calculate sample time
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # Get throttle value from pid controller
        throttle = self.throttle_controller.step(vel_error, sample_time)

        # Calculate brake 
        brake = 0

        # if vehicle is stopped
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            # Brake torque of 700 instead of 400 to avoid rolling
            brake = 700 #N*m - to hold the car in place if we are stopped at a light. Acceleration ~ 1m/s^2

        # if car needs to brake
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            # Consider max deceleration
            decel = max(vel_error, self.decel_limit)
            # Calculate final brake torque
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque N*m

        return throttle, brake, steering
