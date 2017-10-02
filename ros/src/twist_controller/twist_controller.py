
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
  def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle): # , *args, **kwargs
    # contoller parameters: kp, ki, kd
    self.velocity_pid = PID(2.0, 0.0, 0.0)
    self.steer_pid = PID(6.8, 0.0, 0.05)

    self.steer_control = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

    self.steer_lowpass = LowPassFilter(1.1, 0.05)

  def control(self, target_vel_lin, target_vel_ang, cur_vel_lin, cur_vel_ang, time_elapsed):

    vel_error = target_vel_lin - cur_vel_lin

    throttle = self.velocity_pid.step(vel_error, time_elapsed)
    brake = max(0.0, -throttle)
    throttle = max(0.0, throttle)


    target_steer =  self.steer_control.get_steering(target_vel_lin, target_vel_ang, target_vel_lin)
    cur_steer = self.steer_control.get_steering(cur_vel_lin, cur_vel_ang, cur_vel_lin)
    steer = self.steer_pid.step(target_steer - cur_steer, time_elapsed)

    # apply low pass filter
    steer = self.steer_lowpass.filt(steer)
    
    # Return throttle, brake and steer
    return throttle, brake, steer

  def reset(self):
    self.velocity_pid.reset()
    self.steer_pid.reset()