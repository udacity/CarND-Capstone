from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    # same params passing from dwb_node.py
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel,max_steer_angle):

        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # play around with this PID params
        kp = 0.3
        ki = 0.01
        kd = 0.3
        mn = 0. #minimum throttle value
        mx = 0.2 #maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # play around with lowpassfilter
        tau = 0.5 # 1/(2pi*tau) is cutoff frequency
        ts = 0.02 # Sample time
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

        # just to filter out noise from current velocity
        current_vel = self.vel_lpf.filt(current_vel)

        # rospy.logwarn("Angular vel: {0}".format(angular_vel))
        # rospy.logwarn("Target vel: {0}".format(linear_vel))
        # rospy.logwarn("Target angular vel: {0}".format(angular_vel))
        # rospy.logwarn("Current vel: {0}".format(current_vel))
        # rospy.logwarn("Filter vel: {0}".format(self.vel_lpf.get()))
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0 and current_vel < 0.1:
            throttle = 0
            brake = 400  # torque N*m Acceleration - 1m/s^2
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius  # torgue N*m

        return throttle, brake, steering
