import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858

KP = 1.1
KI = 0.01
KD = 0.005
MIN_SPEED = 1.0


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.pid_controller = PID(KP, KI, KD, mn=kwargs['decel_limit'], mx=kwargs['accel_limit'])
        self.yaw_controller = YawController(kwargs['wheel_base'],
                                            kwargs['steer_ratio'],
                                            MIN_SPEED,
                                            kwargs['max_lat_accel'],
                                            kwargs['max_steer_angle'])
        self.low_pass_filter = LowPassFilter(tau=0.5, ts=0.8)

        self.total_mass = kwargs['vehicle_mass'] + kwargs['fuel_capacity'] * GAS_DENSITY
        self.wheel_radius = kwargs['wheel_radius']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.max_throttle = kwargs['max_throttle']

        self.reset()

    def reset(self):
        self.pid_controller.reset()

    def control(self, current_velocity, twist, diff_time):
        target_linear_velocity = twist.linear.x
        target_angular_velocity = twist.angular.z

        steer = self.yaw_controller.get_steering(target_linear_velocity,
                                                   target_angular_velocity,
                                                   current_velocity)

        linear_velocity_diff = target_linear_velocity - current_velocity
        accel = self.pid_controller.step(linear_velocity_diff, diff_time)
        accel = self.low_pass_filter.filt(accel)

        # if target velocity is almost 0, keep full brake
        if target_linear_velocity < 0.1:
            throttle = 0.0
            brake = self.acceleration_to_torque(abs(self.decel_limit))
            return throttle, brake, steer

        if accel > 0.0:
            throttle = accel * self.max_throttle
            brake = 0.0
            #rospy.logwarn("accel: %.3f, throttle: %.3f, brake: %.3f, current velocity: %.3f", accel, throttle, brake, current_velocity)
        elif abs(accel) >= self.brake_deadband:
            throttle = 0.0
            brake = self.acceleration_to_torque(abs(accel))
            #rospy.logwarn("accel: %.3f, throttle: %.3f, brake: %.3f, targtvel: %.3f, currvel: %.3f", accel, throttle, brake, target_linear_velocity, current_velocity)
        else:
            # in the brake deadband, just let the engine brake
            throttle = 0.0
            brake = 0.0

        return throttle, brake, steer

    def acceleration_to_torque(self, acceleration):
        # compute brake torque in Nm
        # the braking force F = ma
        # where m is the total mass and a is the control value
        # in which the control is the torque to be applied by the brake
        return acceleration * self.total_mass * self.wheel_radius
