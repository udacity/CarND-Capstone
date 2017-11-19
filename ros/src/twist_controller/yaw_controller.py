from math import atan
from pid import PID
import rospy

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.ang_pid     = PID(0.2,0.01,0) #PID(24.5,10.6,0)
        self.wheel_base  = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed   = min_speed          # minimum speed to which saturate the steering angle of speed lower than it, used to avoid singularity that will lead to maximum steering angle when the car is slow
        self.max_lat_accel = max_lat_accel  # maximal lateral acceleration allowed

        self.min_angle = -max_steer_angle  
        self.max_angle = max_steer_angle
	#self.max_steer_speed = 8.7

  
    # This method given a desidered radius of motion returns the needed steering angle
    def get_angle(self, radius):
        # atan computes the fron wheels steering angle required
        # multiply for the steer_ratio convert the angle of
        # the wheels in the steering angle 
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    # This method 	
    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        # velocity/angular_speed = curvature radius
        # so if we divide angular_velocity / linear_velocity that are the desidered
        # value we obtain 1/R_desidered
        # Then dividing the current_velocity by the R_desidered we obtain the angular 
        # velocity that have to be imposed at the current time, with the real velocity 
        # and not with the desidered one
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.  #If the car is stopped put the wheel at 0 angle

	# if we are too slow no need to saturate by the maximum lateral acceleration
        if abs(current_velocity) > 0.1:      
            # Compute the maximum angular velocity allowed 
            # lateral_acc = V^2/R  = omega*V   since omega*R = V
            max_yaw_rate = abs(self.max_lat_accel / current_velocity);
            # Saturate the required angular velocity in the allowed range
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))
        return self.get_angle_from_speed(current_velocity,angular_velocity)


    def get_angle_from_speed(self,linear_speed,angular_speed):   
        # Compute the needed steering angle, 
        # Desired radius of curvature = current_velocity/angular_velocity
        # The max(current_velocity, self.min_speed) is done to avoid the singularity
        #that steer the wheels at 90 degreees in the case that the vehicle is stopped
        return self.get_angle(max(linear_speed, self.min_speed) / angular_speed) if abs(angular_speed) > 0. else 0.0 # TODO: check if it is useful to increse the comparison with tha absolute value with a value greater than zero, this will avoid continuous microcorrection of the steer wheel when we are oriented in good way 

    def step(self, linear_velocity, angular_velocity, current_velocity, current_angular_velocity,delta_t):
        # Evaluate the set point of the steering angle
        desidered_steer_angle = self.get_steering(linear_velocity, angular_velocity, current_velocity)
        measured_steer_angle  = self.get_steering(current_velocity, current_angular_velocity, current_velocity) #self.get_angle_from_speed(current_velocity,current_angular_velocity)

        # Compute the traking error
        error = desidered_steer_angle - measured_steer_angle
        rospy.loginfo('steer_angle_c {0}'.format(error))
        requested_steer_angle =  desidered_steer_angle + 0*self.ang_pid.step(error, delta_t)

	return requested_steer_angle

  
    def reset(self):  
        self.ang_pid.reset()
