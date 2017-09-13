from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_SPEED_MPH = 50


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.gainp=1.
        self.gaini=0.
        self.gaind=0.
        #pid will use m/s, all velocities must be converted from mph to m/s
        self.max_speed_mps = MAX_SPEED_MPH*ONE_MPH
        self.min_speed_mps = 0
        
        self.throttle = 0.
        self.brake = 0.
        self.steer = 0.
        
        self.speed_pid = PID(self.gainp,self.gaini,self.gaind, -1.0 , 1.0)
        self.yaw_con = YawController(1,1,1,1,1)

    def control(self, target_speed_mps, current_speed_mps, sample_time_s=.02, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        effort = self.speed_pid.step(target_speed_mps-current_speed_mps, sample_time_s)
        if effort < 0:
            self.throttle = 0
            self.brake = abs(effort)
        else:
            self.throttle = effort
            self.brake = 0
        return self.throttle, self.brake, self.steer

    def configure_yaw_controller(self, wheel_base,steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.yaw_con = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)