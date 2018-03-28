from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import time
import rospy

ONE_MPH = 0.44704
MAX_SPEED = 50.0

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
         # kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM
        self.throttle_pid = PID(0.5, 0.00001, 0.0)
        self.brake_pid    = PID(0.6, 0.00001, 0.0)
        self.steer_pid    = PID(0.8, 0.00001, 0.0)
        # wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle
        self.yaw_control = YawController(kwargs['wheel_base'], kwargs['steer_ratio'],
                                         kwargs['min_speed'], kwargs['max_lat_accel'],
                                         kwargs['max_steer_angle']
                                         )
        self.last_t = None
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.filter = LowPassFilter(0.2,0.1)

    def control(self, target_v, target_w, current_v, dbw_enabled, cte):
        # TODO: Change the arg, kwarg list to suit your needs
        print 'target_v', target_v
        print "current_v", current_v
        if self.last_t is None or not dbw_enabled:
            self.last_t = rospy.get_time()
            return 0.0, 0.0, 0.0

        dt = rospy.get_time() - self.last_t

        error_v = min(target_v.x, MAX_SPEED*ONE_MPH) - current_v.x
        print "first vel error", error_v
        # error_v = max(self.decel_limit*dt, min(self.accel_limit*dt, error_v))
        throttle = self.throttle_pid.step(error_v, dt)
        print "second vel error", error_v
        throttle = max(0.0, min(1.0, throttle))
        if error_v < 0:
            brake = self.brake_pid.step(error_v, dt)
            brake = max(0.0, min(1.0, brake))
            throttle = 0.0
        else:
            brake = 0.0

        steer = self.yaw_control.get_steering(target_v.x, target_w.z, current_v.x) + self.steer_pid.step(cte, dt)
        steer = max(-abs(self.max_steer_angle), min(abs(self.max_steer_angle), steer))

        steer = self.filter.filt(steer)
        self.last_t = time.time()

        return throttle, brake, steer
