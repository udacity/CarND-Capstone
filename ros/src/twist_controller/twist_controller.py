from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, vehicle_mass):
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_base

        self.yawcontroller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.set_controllers()
        pass

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled, time_elapsed):
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.set_controllers()
            return 0.0, 0.0, 0.0

        # throttle and brake controllers
        linear_velocity_error = linear_velocity - current_velocity
        if linear_velocity_error > 0.0:
            throttle = self.pid_throttle.step(linear_velocity_error, time_elapsed)
            brake = 0.0
        else:
            throttle = 0.0
            if linear_velocity < 0.2:
                brake = (self.vehicle_mass * self.wheel_radius) / 2.0
            else:
                brake = -(self.vehicle_mass * self.wheel_radius * linear_velocity_error)

        # steering controller
        steer = self.yawcontroller.get_steering(linear_velocity, angular_velocity, current_velocity)
        steer = self.lowpass_flt.filt(steer)

        return throttle, brake, steer

    def set_controllers(self):
        self.pid_throttle = PID(0.35, 0.0, 0.0, 0.0, 1.0)
        self.pid_brake = PID(0.30, 0.0, 0.0, 0.0, 1.0)
        self.lowpass_flt = LowPassFilter(0.2, 1.0)
