"""Throttle, brake, and steering control."""
from math import atan
from lowpass import LowPassFilter
from pid import PID
from rospy import loginfo

class Controller(object):
    """
    Class to calculate throttle, bake, and steering control.

    Uses PID controllers for steering and throttle.
    """

    def __init__(self, kp_vel, kp_throttle, ki_throttle, min_speed,
                 vehicle_mass, fuel_capacity, brake_deadband,
                 decel_limit, accel_limit,
                 wheel_radius, wheel_base,
                 steer_ratio, max_lat_accel, max_steer_angle, rate):
        """
        Constructor.

        Set up gains and initialize internal state; initialize PID controllers.
        """
        self.i_error = 0
        self.kp_vel = kp_vel
        self.kp_throttle = kp_throttle
        self.ki_throttle = ki_throttle
        self.last_cur_lin_vel = 0
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband
        self.steer_ratio = steer_ratio
        self.wheel_base = wheel_base
        self.cur_acc = 0
        self.rate = rate
        self.accel_filter = LowPassFilter(0.2, 1./self.rate)
        self.steer_pid = PID(0.5, 0.001, 0.0)
        self.throttle_pid = PID(kp_throttle, ki_throttle, 0.0)

    def control(self, cmd_lin_vel, cmd_ang_vel, cur_lin_vel, cur_ang_vel, delta_t, dbw_enabled):
        """
        Control update.

        This function should be called at the rate given in the constructor.
        Returns a tuple of (throttle, brake, steering)
        """
        brake = 0.
        throttle = 0.

        loginfo("cmd_ang_vel: %f", cmd_ang_vel)
        loginfo("cmd_lin_vel: %f", cmd_lin_vel)

        steer_error = cmd_ang_vel - cur_ang_vel

        steering = 10*atan(self.wheel_base * cmd_ang_vel / (cmd_lin_vel+0.01)) * self.steer_ratio
        cmd_acc = self.kp_vel * (cmd_lin_vel - cur_lin_vel)

        if cmd_acc > self.accel_limit:
            cmd_acc = self.accel_limit
        elif cmd_acc < self.decel_limit:
            cmd_acc = self.decel_limit

        loginfo("cmd_acc: %f", cmd_acc)
        loginfo("cur_acc: %f", self.cur_acc)

        if not dbw_enabled:
            self.steer_pid.reset()
            self.throttle_pid.reset()

        throttle_error = cmd_acc

        brake = -throttle_error*self.vehicle_mass*self.wheel_radius
        throttle = self.throttle_pid.step(throttle_error, delta_t)

        if cmd_acc < 0:
            if brake < self.brake_deadband:
                brake = 0.
            throttle = 0.
        else:
            brake = 0.

        loginfo("throttle: %f", throttle)

        return throttle, brake, steering
