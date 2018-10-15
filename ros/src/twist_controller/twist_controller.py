import rospy

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

Kp = 0.23
Ki = 0.1
Kd = 0.

Mn = 0.0
Mx = 0.2

Tau = 0.5
Ts = 0.02


class Controller(object):
    def __init__(self, *args, **kwargs):
        params = args[0]
        self.steer = YawController(params['wheel_base'],
                                 params['steer_ratio'],
                                 params['min_speed'],
                                 params['max_lat_accel'],
                                 params['max_steer_angle'])

        self.throttle = PID(kp=Kp, ki=Ki, kd=Kd, mn=Mn, mx=Mx)

        self.vel_lpf = LowPassFilter(Tau, Ts)

        self.params = params
        self.last_vel = 0.
        self.last_time = rospy.get_time()

    def control(self, current_vel, angular_vel, linear_vel, dbw):
        # Return throttle, brake, steer
        if not dbw:
            self.throttle.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        val_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle.step(val_error, sample_time)
        brake = 0.

        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0.
            brake = 700  # N*m
        elif throttle < 0.1 and val_error < 0:
            throttle = 0.
            decel = max(val_error, self.params['decel_limit'])
            brake = abs(decel) * self.params['vehicle_mass'] * self.params['wheel_radius']

        return throttle, brake, steering
