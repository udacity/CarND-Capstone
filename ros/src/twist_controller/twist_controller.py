
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from yaw_controller import YawController
from pid import PID

class Controller(object):
    def __init__(self, *args, **kwargs):

        self.__dict__.update(kwargs)

        kp = -0.2
        ki = -0.05
        kd = -3.0
        self.max_steer_angle = 0.43
        self.pidcontroller = PID(kp, ki, kd, -self.max_steer_angle, self.max_steer_angle)

        min_speed = 0.0
        self.yawcontroller = YawController(self.wheel_base, self.steer_ratio, min_speed, self.max_lat_accel, self.max_steer_angle)

        self.pedal = 1.0

    def control(self, *args): #, **kwargs):

        # input arguments
        cte = args[0]
        delta_time = args[1]
        linear_velocity = args[2]
        angular_velocity = args[3]
        current_velocity = args[4]

        # PID control
        steer_pid = self.pidcontroller.step(cte, delta_time)

        # YAW control
        steer_yaw = 0 #self.yawcontroller.get_steering(linear_velocity, angular_velocity, current_velocity)

        steer = steer_pid + steer_yaw

        throttle = 0.0
        brake = 0.0

        if (current_velocity < linear_velocity):
            self.pedal += 0.0
        else:
            self.pedal -= -0.0

        if (self.pedal>0):
            throttle = self.pedal
        else:
            brake = -self.pedal

        # print 'cte ' + str(cte) + ' -> steer ' + str(steer_pid) + ' + ' + str(steer_yaw) + ' pedal ' + str(self.pedal)
        # + '. angular_velocity = ' + str(angular_velocity)

        return throttle, brake, steer
