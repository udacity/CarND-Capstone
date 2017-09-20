from pid import *
from yaw_controller import YawController
from math import sqrt
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.kp = args[0]
        self.ki = args[1]
        self.kd = args[2]
        # controller parameters
        self.accel_limit = args[3]
        self.decel_limit = args[4]
        self.max_steer_angle = args[5]
        self.sample_time = args[6]
        self.vehicle_mass = args[7]
        self.radius_wheel = args[8]
        self.wheel_base = args[9]
        self.steer_ratio = args[10]
        self.max_lat_accel = args[11]
        self.min_speed = 0.2 # some random min speed I made up 
        # define two controller for throttle and brake
        self.u_throttle = PID(self.kp, self.ki, self.kd, mn=0, mx=self.accel_limit)
        self.u_brake = PID(self.kp, self.ki, self.kd, mn=self.decel_limit, mx=0)
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 
                                            self.min_speed, self.max_lat_accel,
                                            self.max_steer_angle)
        #self.u_steer = PID(self.kp, self.ki, self.kd, mn=0, mx=self.mx_turn)
        # reset controller
        self.u_throttle.reset()
        self.u_brake.reset()
        #self.u_steer.reset()

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        target_vel = args[0]
        target_ang = args[1]
        current_vel = args[2]
        dbw_en = args[3]

        
        tar_vel_mag = sqrt((target_vel.x)**2 + 
                              (target_vel.y)**2 +
                              (target_vel.z)**2)
        cur_vel_mag = sqrt((current_vel.x)**2 +
                               (current_vel.y)**2 +
                               (current_vel.z)**2)
        
        vel_diff = tar_vel_mag - cur_vel_mag
        if vel_diff >= 0:
            throttle = self.u_throttle.step(vel_diff, self.sample_time)
            brake = 0
        else:
            # brake should be calculated with desired acceleration, weight
            # and wheel radius: F = ma, torque = F*radius_wheel
            throttle = 0;
            desired_a = self.u_brake.step(vel_diff, self.sample_time)
            force = desired_a*self.vehicle_mass
            brake = force*self.radius_wheel

        # steer signal should come from the yaw_controller
        steer = self.yaw_controller.get_steering(tar_vel_mag, target_ang.z, cur_vel_mag)


        return throttle, brake, steer
