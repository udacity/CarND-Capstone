from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy 

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
    	accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 1.5, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0  # min throttle value
        mx = 0.5  # max throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        self.steer_controller = PID(0.2, 0.10, 0.45, -max_steer_angle, max_steer_angle)

        tau = 0.2  # 1 / (2*pi* tau) = cuttoff freq
        ts = 0.02  # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        tau_cte = 0.2  # 1 / (2*pi* tau) = cuttoff freq
        ts_cte = 0.02  # sample time, f
        self.cte_lpf = LowPassFilter(tau_cte, ts_cte)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel, cte):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
        	self.throttle_controller.reset()
        	self.steer_controller.reset()
        	return 0.0, 0.0, 0.0

        current_vel = self.vel_lpf.filt(current_vel)

        predictive_steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        cte_lpf = self.cte_lpf.filt(cte)
        pid_steering = self.steer_controller.step(cte_lpf, sample_time)

        steering = 0.4 * predictive_steering + 0.6 * pid_steering

        throttle = self.throttle_controller.step(vel_error, sample_time)

        brake = 0

        if linear_vel == 0.0 and current_vel < 0.1:
        	throttle = 0
        	brake = 400   #N*m - hold the car when stopped at a light

        elif throttle < 0.1 and vel_error < 0:
        	throttle = 0
        	decel = max(vel_error, self.decel_limit)
        	brake = abs(decel) * self.vehicle_mass * self.wheel_radius  #torque = N*m

        return throttle, brake, steering
