import rospy
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, kp, ki, kd, vc):

        self.kp = kp
    	self.ki = ki
    	self.kd = kd
    	self.max_accel = vc.accel_limit
    	self.max_decel = vc.accel_limit

        self.linear_controller = PID(self.kp, self.ki, self.kd, mn=self.max_decel, mx=self.max_accel)
        self.angular_controller = YawController(vc.wheel_base, vc.steer_ratio, vc.min_speed, vc.max_lat_accel, vc.max_steer_angle)

        # TODO this is not so good
        self.last_run_time = rospy.get_time()

    def reset(self):
        self.linear_controller.reset()
        # self.angular_controller.reset() # TODO there is no reset command here, is this ok?

    def control(self, current_velocity, twist_cmd):

        time_now = rospy.get_time()
        time_elapsed = time_now - self.last_run_time
        linear_reference_velocity = abs(twist_cmd.twist.linear.x) # accounting for negative twist_cmds
        angular_reference_velocity = twist_cmd.twist.angular.z
        linear_error = linear_reference_velocity - current_velocity.twist.linear.x

        linear = self.linear_controller.step(linear_error, time_elapsed)
        angular = self.angular_controller.get_steering(current_velocity.twist.linear.x, angular_reference_velocity, linear_reference_velocity)

        self.last_run_time = time_now

        throttle = linear if linear > 0.0 else 0.0
        brake = -linear if linear <= 0.0 else 0.0

        return throttle, brake, angular
