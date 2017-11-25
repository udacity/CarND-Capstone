
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
        self.steer_ratio = steer_ratio
        self.wheel_base = wheel_base
        self.yaw_ctrl = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.acc_pid = PID(-1,-0.1,-0.2)
        
        pass

    def control(self, cmd_linear, cmd_angular, cur_linear,cur_angular, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        delta_t = (rospy.Time.now() - self.time).to_sec()
        self.time = rospy.Time.now()

        acc = self.acc_pid.step(cur_linear.x - cmd_linear.x, delta_t)

        ang = self.yaw_ctrl.get_steering(max(cmd_linear.x,0), cmd_angular.z, cur_linear.x)
		
        #theta = ang/self.steer_ratio
        #curvature = tan(theta)/self.wheel_base
        #predicted_angular = cur_linear.x*curvature
        #rospy.loginfo('linear: cmd {0} cur {1}  angular: cmd {2} cur {3} pred {4}  steer: {5}'.format(cmd_linear.x,cur_linear.x,cmd_angular.z,cur_angular.z,predicted_angular,ang))
        if not dbw_enabled:
            self.acc_pid.reset()


        return max(acc,0), -200. * min(acc,0), ang

