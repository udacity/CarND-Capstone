
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from yaw_controller import YawController
from pid import PID


class TwistController(object):
    def __init__(self, vehicle_mass, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        
        #vehicle parameters
        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        
        #PID gain for throttle control
        self.kp = 1
        self.kd = 1
        self.ki = 0.1
        
        #PID gain for brake control        
        self.kp_brake = 1
        self.kd_brake = 1
        self.ki_brake = 0.1
        
        self.min_speed = 0.0
        
        self.pid_controller = PID(self.kp, self.ki, self.kd, 0, self.accel_limit)
        
        self.brake_controller = PID(self.kp_brake, self.ki_brake, self.kd_brake, 0, self.decel_limit)
        
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)

        


    def control(self, velocity_cmd, current_velocity, angular_velocity_cmd, dbw_enabled):
        
        if dbw_enabled:
            steer = self.yaw_controller.get_steering(velocity_cmd, angular_velocity_cmd, current_velocity)
            if velocity_cmd > current_velocity:
                error = velocity_cmd - current_velocity
                throttle = self.pid_controller.step(error, 0.02)
                brake = 0.0
                self.brake_controller.reset()
            else:
                error = current_velocity - velocity_cmd
                brake = brake.pid_controller.step(error, 0.02)
                throttle = 0.0
                self.pid_controller.reset()
        else:
            self.pid_controller.reset()
            self.brake_controller.reset()
            throttle = 0.0
            brake = 0.0
            steer = 0.0
        # TODO: Change the arg, kwarg list to suit your needs
        return throttle, brake, steer
