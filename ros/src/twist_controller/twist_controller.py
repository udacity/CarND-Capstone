# import pid, lowpass and yaw_controller
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
	self.last_time = None
	wheel_base = kwargs['wheel_base']
	steer_ratio = kwargs['steer_ratio']
 	min_speed = 0.
	max_lat_accel = kwargs['max_lat_accel']
        accel_limit = kwargs['accel_limit']
	decel_limit = kwargs['decel_limit']
	self.max_steer_angle = kwargs['max_steer_angle']
	self.brake_deadband = kwargs['brake_deadband']
        
	self.control_pid = PID(-0.1, -0.002, -12, decel_limit, accel_limit)  
	self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, self.max_steer_angle)
	self.lowpassfilter  = LowPassFilter(0.7, 0.1)


    def control(self, twist_cmd, current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
	proposed_linear_velocity = twist_cmd.twist.linear.x
	proposed_angular_velocity = twist_cmd.twist.angular.z
 	current_linear_velocity = current_velocity.twist.linear.x
        current_angular_velocity = current_velocity.twist.angular.z
	
	# CTE: error_linear_velocity
	error_linear_velocity = current_linear_velocity - proposed_linear_velocity 
  
        if(dbw_enabled == False):
            self.control_pid.reset()
        
	if(dbw_enabled and self.last_time):
            # get time interval
            time = rospy.get_time()
            delta_time = time - self.last_time
            self.last_time = time
	    # get pid velocity
	    pid_control = self.control_pid.step(error_linear_velocity, delta_time)
	    #pid_control = 1.0 - (current_linear_velocity/proposed_linear_velocity)
            pid_control = max(-1., min(pid_control, 1.))

            throttle = max(0.0, pid_control)
            brake = max(0.0, (-pid_control + self.brake_deadband))  # 
	    
	    steer = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)
	    steer = max(-self.max_steer_angle, min(steer, self.max_steer_angle))
	    
	else:
	    # reset self.last_time
	    self.last_time = rospy.get_time()
	    throttle = 1.
            brake = 0.
            steer = 0.
	
	steer = self.lowpassfilter.filt(steer)
        # Return throttle, brake, steer
        return throttle, brake, steer
