from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

sign = lambda x: (1, -1)[x < 0]

class TwistController(object):
    def __init__(self, accel_limit,
                       decel_limit,
                       yaw_dot_limit,
                       brake_torque,
                       wheel_base,
                       steer_ratio,
                       max_lat_accel,
                       max_steer_angle):

        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.yaw_dot_limit = yaw_dot_limit
        self.brake_torque = brake_torque

        self.accel_pid = PID(0.40,   # kp
                             0.00,   # ki
                             0.00,   # kd
                             self.decel_limit,
                             self.accel_limit)

        self.yaw_controller = YawController(wheel_base, steer_ratio, 1,
                                            max_lat_accel, max_steer_angle)

        self.last_accel = 0
        self.accel_lerp_ratio = 0.75
        self.last_steer = 0
        self.steer_lerp_ratio = 0.9

    def LERP(self, a, b, ratio):
        return a*ratio + (1.-ratio) * b

    def control(self, target_velocity, target_yaw_dot, current_velocity, dt):
        # update acceleration pid
        d_velocity = target_velocity - current_velocity
        accel = self.accel_pid.step(d_velocity, dt)

        # smooth acceleration with LERP function
        accel = self.LERP(self.last_accel, accel, self.accel_lerp_ratio)
        self.last_accel = accel

        if accel<0:
            accel *= self.brake_torque


        steer = self.yaw_controller.get_steering(target_velocity,
                                                 target_yaw_dot,
                                                 current_velocity)

        # smooth steering with LERP function
        steer = self.LERP(self.last_steer, steer, self.steer_lerp_ratio)
        self.last_steer = steer

        return accel, steer

    def reset(self):
        self.accel_pid.reset()
        self.last_accel = 0
        self.last_steer = 0
