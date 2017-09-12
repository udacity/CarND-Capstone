import rospy
import pid
import lowpass
import yaw_controller

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, kp, ki, kd,wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.pidc = pid.PID(kp,ki,kd)
        self.yawc = yaw_controller.YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        pass

    def control(self, pl, pa, cl, elapsed_time, dbw):
        throttle = 0.00001
        steer = 0.0
        dbw = True
        if not dbw:
            self.pidc.reset()
        elif elapsed_time:
            throttle = self.pidc.step(pl-cl,elapsed_time)
            
        steer = self.yawc.get_steering(pl,pa,cl)    
        return throttle, 0.0, steer
