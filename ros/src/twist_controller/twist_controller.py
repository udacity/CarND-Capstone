from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Just copied walk through, probably should modify later
        self.yaw_controller = YawController(kwargs['wheel_base'],
                                            kwargs['steer_ratio'],
                                            0.1,
                                            kwargs['max_lat_accel'],
                                            kwargs['max_steer_angle'])
        kp = 0.3
        ki = 0.1
        kd = 0
        mn = 0
        mx = 0.2
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']

        self.last_time = rospy.get_time()



    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs

        if not kwargs['dbw_enabled']:
           self.throttle_controller.reset()
           return 0, 0, 1

        current_vel = self.vel_lpf.filt(kwargs['current_vel'])
        steering = self.yaw_controller.get_steering(kwargs['linear_vel'], kwargs['angular_vel'], kwargs['current_vel'])

        vel_error = kwargs['linear_vel'] - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if kwargs["linear_vel"] == 0 and current_vel < 0.1:
            throttle = 0
            brake = 400
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        return throttle, brake, steering