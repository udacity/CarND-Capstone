from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_SPEED_MPH = 50


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.gainp=1.0
        self.gaini=0.
        self.gaind=0.
        #pid will use m/s, all velocities must be converted from mph to m/s
        self.max_speed_mps = MAX_SPEED_MPH*ONE_MPH
        self.min_speed_mps = 0
        
        self.throttle = 0.
        self.brake = 0.
        self.steer = 0.
        
        self.speed_pid = PID(self.gainp,self.gaini,self.gaind,self.max_speed_mps,self.min_speed_mps)
        
        self.yaw_con = YawController(1, 1, 1, 1, 1)
        

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        return self.throttle, self.brake, self.steer

    def configure_yaw_controller(self, wheel_base,steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.yaw_con = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)