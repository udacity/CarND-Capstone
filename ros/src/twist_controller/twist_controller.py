
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle): # , *args, **kwargs
        # TODO: Implement
        # contoller parameters: kp, ki, kd
        self.velocity_pid = PID(0.6, 0.05, 0.1)
        # self.steer_pid = PID(6.0, 0.3, 1.0)

        self.steer_control = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # self.steer_lowpass = LowPassFilter(0.8, 0.2)

    def control(self, linear_velocity, angular_velocity, current_velocity, vehicle_cur_ang, time_elapsed):

        # TODO: Change the arg, kwarg list to suit your needs
        brake = 0

        # might need to use the yaw_controller to get the steer angle: needed inputs are wheel_base, steer_ratio, min_speed, max_lat_accel and max_steer_angle to initialize class, then linear_velocity, angular_velocity and current_velocity to get steering.
        vel_error = 30*ONE_MPH - current_velocity
        throttle = self.velocity_pid.step(vel_error, time_elapsed)
        if throttle < 0:
        	brake = throttle
        	throttle = 0
        steer = self.steer_control.get_steering(linear_velocity, angular_velocity, current_velocity)
        steer_kp = 0.6
        steer += steer_kp*(angular_velocity - vehicle_cur_ang)
        # steer = self.steer_pid.step(cte, time_elapsed)
        # if abs(steer) > 5.0:
        #   self.steer_pid.reset()
        # steer = self.steer_lowpass.filt(steer)
        
        # Return throttle, brake, steer
        return throttle, brake, steer
