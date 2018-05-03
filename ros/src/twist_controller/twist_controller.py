import rospy

from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,
                 decel_limit, accel_limit, wheel_radius, wheel_base,
                 steer_ratio, max_lat_accel, max_steer_angle):

        # Initialize yaw controller
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1,
                                            max_lat_accel, max_steer_angle)
        # Initialize PID controller
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0.  # min throttle value
        mx = 0.2 # max throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # Initialize low pass filter
        tau = 0.5 # 1/(2pi * tau) = cutoff frequency
        ts = .02  # sample time
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

        # ropsy.logwarn("Angular vel: {}".format(angular_vel))
        # rospy.logwarn("Target velocity: {}".format(linear_vel))
        # rospy.logwarn("Target angular velocity: {}\n".format(angular_vel))
        # rospy.logwarn("Current velocity: {}".format(current_vel))
        # rospy.logwarn("Filtered velocity: {}".format(self.vel_lpf.get()))

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
    
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0. and current_vel < 0.1:
            # if the speed is really slow, we want to stop the car completely
            throttle = 0
            brake = 400
        elif throttle < .1 and vel_error < 0:
            # set the brake baed on speed error
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque N*m
        
        return throttle, brake, steering
