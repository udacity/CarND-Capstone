import rospy
from std_msgs.msg import Float64
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, kp, ki, kd, vc):

        self.kp = kp
    	self.ki = ki
    	self.kd = kd
    	self.max_accel = vc.accel_limit
    	self.max_decel = vc.decel_limit
        self.brake_deadband = vc.brake_deadband

        self.linear_controller = PID(self.kp, self.ki, self.kd, mn=self.max_decel, mx=self.max_accel)
        self.angular_controller = YawController(vc.wheel_base, vc.steer_ratio, vc.min_speed, vc.max_lat_accel, vc.max_steer_angle)

        self.v_error_pub = rospy.Publisher('/velocity_error', Float64, queue_size=1)
        self.v_filter_pub = rospy.Publisher('/velocity_filter', Float64, queue_size=1)

        # self.linear_filter = LowPassFilter(tau=0.5, ts=1.5) # suggest some values here...
        # self.linear_filter = LowPassFilter(tau=0.5, ts=1.0) # suggest some values here...
        # self.linear_filter = LowPassFilter(tau=5.5, ts=5.0) # seems ok
        self.linear_error_filter = LowPassFilter(tau=4.5, ts=5.0) # seems ok

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

        self.v_error_pub.publish(linear_error)
        linear_error = self.linear_error_filter.filt(linear_error) ##############################################
        self.v_filter_pub.publish(linear_error)

        linear = self.linear_controller.step(linear_error, time_elapsed)

        angular = self.angular_controller.get_steering(linear_reference_velocity, angular_reference_velocity, current_velocity.twist.linear.x)

        rospy.loginfo("""Velocity Ref: {} - Curr: {} - Err: {}""".format(linear_reference_velocity, current_velocity.twist.linear.x, linear_error))

        self.last_run_time = time_now

        # linear = self.linear_filter.filt(linear) ##############################################

        throttle = linear if linear > 0.0 else 0.0
        brake = -linear-self.brake_deadband if linear <= 0.0 else 0.0

        return throttle, brake, angular
