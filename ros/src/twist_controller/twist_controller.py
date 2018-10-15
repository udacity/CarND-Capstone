import rospy

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

Kp = 0.3
Ki = 0.1
Kd = 0.

Mn = 0.0
Mx = 0.2

Tau = 0.5
Ts = 0.2


class Controller(object):
    def __init__(self, *args, **kwargs):
        params = args[0]

        self.yaw_controller = YawController(params['wheel_base'],
                                 params['steer_ratio'],
                                 params['min_speed'],
                                 params['max_lat_accel'],
                                 params['max_steer_angle'])

        self.decel_limit = params['decel_limit']
        self.vehicle_mass = params['vehicle_mass']
        self.wheel_radius = params['wheel_radius']

        self.throttle_controller = PID(kp=Kp, ki=Ki, kd=Kd, mn=Mn, mx=Mx)
        self.vel_lpf = LowPassFilter(Tau, Ts)

        self.last_vel = 0.
        self.last_time = rospy.get_time()

    def control(self, current_vel, angular_vel, linear_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs

        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)
        current_time = rospy.get_time()

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        # rospy.loginfo('%s  %s',angular_vel,steering)
        # rospy.loginfo('%s',steering)

        vel_error = linear_vel - current_vel
        sample_time = current_time - self.last_time

        self.last_vel = current_vel
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 400  # N*m-to hold the car in place if we are stopped at light.Acceleration-1m/s^2
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius  # Torque N*m

        return throttle, brake, steering
