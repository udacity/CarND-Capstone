from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, rate, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius,
                 wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, max_throttle_percent):
        self.pid = PID(1., 0, 0)
        self.yaw = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        #self.filter_throttle = LowPassFilter(0.01, 1. / rate)  # 2/3 of new value, 1/3 of prev value (50Hz)
        #self.filter_brake = LowPassFilter(0.01, 1. / rate)  # 2/3 of new value, 1/3 of prev value (50Hz)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.max_throttle = max_throttle_percent

        mass = self.vehicle_mass + GAS_DENSITY * self.fuel_capacity
        self.max_brake = mass * self.wheel_radius * (-self.decel_limit)

    def control(self, target_linear, target_angular, current_linear, step):
        throttle = 0.
        brake = 0.
        value = self.pid.step(target_linear - current_linear, step)
        if value > 0.:
            throttle = self.max_throttle * math.erf(value)
            #throttle = self.filter_throttle.filt(throttle)
        else:
            brake = self.max_brake * math.erf(-value)
            #brake = self.filter_brake.filt(brake)

        steer = self.yaw.get_steering(target_linear, target_angular, current_linear)

        return throttle, brake, steer
