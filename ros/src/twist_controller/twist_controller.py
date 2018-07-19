import rospy
from yaw_controller import YawController
from lowpass import LowPassFilter
from twiddle import Twiddle

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius,
                 wheel_base, steer_ratio, max_lat_accel, max_steer_angle, tuning_active):

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # When parameter tuning_active is false, then Twiddle behaves just like a PID controller.
        self.throttle_controller = Twiddle(
            coeffs=[1.1999999999999993, -0.021952551922336052, -0.003874204890000004],
            delta_coeffs=[0.004031356972346669, 0.00041132098483284045, 0.0002503155504993245],
            mn=0.0, mx=1.0, active=tuning_active)
        self.vel_lpf = LowPassFilter(tau=0.5, ts=0.02)
        self.last_time = rospy.get_time()

    def reset(self):
        self.throttle_controller.reset()  # Reset PID controller

    def control(self, linear_vel, angular_vel, current_vel):

        # The LP-filter helps produce a more reliable velocity value from the noisy input signal
        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        vel_error = linear_vel - current_vel
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0
            brake = 700  # Torque (Nm) needed to keep Carla in place when stopped
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0.0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius  # Brake torque in Nm

#        rospy.loginfo("actualSpeed=%s, targetSpeed=%s, error=%s", current_vel, linear_vel, vel_error)
        return throttle, brake, steering

    def set_next_params(self):
        self.throttle_controller.set_next_params()
