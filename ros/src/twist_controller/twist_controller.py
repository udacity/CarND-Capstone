
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle): # , *args, **kwargs
        # TODO: Implement
        # contoller parameters: kp, ki, kd
        self.velocity_pid = PID(1.0, 0.05, 0.2)
        # self.steer_pid = PID(6.0, 0.3, 1.0)
        # self.steer_pid = PID(0.6, 0.7, 0.4) 
        self.steer_pid = PID(6.0, 0.0, 0.00002)

        self.steer_control = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.steer_lowpass = LowPassFilter(0.4, 0.1)

    def control(self, target_vel_lin, target_vel_ang, cur_vel_lin, cur_vel_ang, time_elapsed):

        # TODO: Change the arg, kwarg list to suit your needs
        brake = 0

        # might need to use the yaw_controller to get the steer angle: needed inputs are wheel_base, steer_ratio, min_speed, max_lat_accel and max_steer_angle to initialize class, then linear_velocity, angular_velocity and current_velocity to get steering.
        
        vel_error = target_vel_lin - cur_vel_lin

        throttle = self.velocity_pid.step(vel_error, time_elapsed)
        brake = max(0.0, -throttle) + 0.2
        throttle = max(0.0, throttle)


        # cur_steer = self.steer_control.get_steering(target_vel_lin, target_vel_ang, cur_vel_lin)

        target_steer =  self.steer_control.get_steering(target_vel_lin, target_vel_ang, target_vel_lin)
        cur_steer = self.steer_control.get_steering(cur_vel_lin, cur_vel_ang, cur_vel_lin)
        steer = self.steer_pid.step(target_steer - cur_steer, time_elapsed)

        # steer = self.steer_lowpass.filt(steer)
        
        # Return throttle, brake, steer
        return throttle, brake, steer
