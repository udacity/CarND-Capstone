
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from geometry_msgs.msg import Vector3
from yaw_controller import YawController
from pid import PID
from math import tan, sqrt
import rospy

class Controller(object):
    def __init__(self,wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        R_min = wheel_base/tan(max_steer_angle/steer_ratio)  # minimun radius of curvature followable by the car
        min_speed = sqrt(max_lat_accel*R_min)    #with this speed we garantee to use all the steering range
        self.time = rospy.Time.now()
        self.ang_pid = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.acc_pid = PID(-1,-0.1,-0.2)
        #self.ang_pid = PID(-2,-0.2,-0.4)
        pass

    def control(self, cmd_linear, cmd_angular, cur_linear, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        delta_t = (rospy.Time.now() - self.time).to_sec()
        self.time = rospy.Time.now()

        acc = self.acc_pid.step(cur_linear.x - cmd_linear.x, delta_t)
        #ang = self.ang_pid.step(-cmd_angular.z, delta_t)
        ang = self.ang_pid.step(cmd_linear.x, cmd_angular.z, cur_linear.x, cur_linear.z,delta_t)

        if not dbw_enabled:
            self.acc_pid.reset()
            self.ang_pid.reset()

        return max(acc,0), -200. * min(acc,0), ang

