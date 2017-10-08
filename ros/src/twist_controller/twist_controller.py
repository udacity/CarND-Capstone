from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import math
import rospy
from styx_msgs.msg import Lane, Waypoint
from geometry_msgs.msg import PoseStamped, Pose

GAS_DENSITY = 2.858


class Controller(object):
    def __init__(self, *args, **params):
           # Parameters for the Vehicle
        self.vehicle_mass = params['vehicle_mass']
        self.fuel_capacity = params['fuel_capacity']
        self.brake_deadband = params['brake_deadband']
        # Acceleration and Deacceleration limits
        self.decel_limit = params['decel_limit']
        self.accel_limit = params['accel_limit']
        # Wheel Paramaters
        self.wheel_radius = params['wheel_radius']
        self.wheel_base = params['wheel_base']
        self.steer_ratio = params['steer_ratio']
        # Maximum Value for Laterial Acceleation and Steering Angle
        self.max_lat_accel = params['max_lat_accel']
        self.max_steer_angle = params['max_steer_angle']  
        # Brake Constant calculated for Reduced processing in the control loop 
        self.brake_constant = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius
        self.current_dbw_enabled = False
        self.min_speed = 4
        # Controller for Managing the Speed
        self.linear_pid = PID(0.9, 0.0004, 0.06, self.decel_limit, self.accel_limit)
        # Low Pass Filter Values Identified Empirically
        self.low_pass_filter_correction = LowPassFilter(0.19, 0.09)
        # Controller for Managing the Yaw
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle )
        self.previous_time = None
        self.loop_freq = params['loop_freq']
        pass
    
    
    
    def control(self, proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity, dbw_enabled):
        self.check_dbw_status(dbw_enabled)
        # Calculate Velocity Error between Proposed and Current
        linear_velocity_error = proposed_linear_velocity - current_linear_velocity
        current_time = rospy.get_time() 
        # Store Time Elapsed between current time and previous time we were here
        if self.previous_time: 
            dt = current_time - self.previous_time 
        else :
            # Default to using controller loop frequency
            dt = 1./self.loop_freq
        self.previous_time = current_time
        # Apply a PID controller
        velocity_correction = self.linear_pid.step(linear_velocity_error, dt)
        # Apply a Low Pass Filter
        velocity_correction = self.low_pass_filter_correction.filt(velocity_correction)
        # If car is slowing down then Apply Deceleration value to the velocity corrections
        if abs(proposed_linear_velocity)<0.01 and abs(current_linear_velocity) < 0.3:
            velocity_correction = self.decel_limit
            
        # Throttle Values are equal to how much velocity needs to be corrected
        throttle = velocity_correction
        brake = 0.
        if throttle < 0.:
            if throttle < self.decel_limit:
                throttle = self.decel_limit
            decel = abs(throttle)
            if decel > self.brake_deadband:
                brake = self.brake_constant * decel 
            else:
                brake = 0.
            throttle = 0.
        elif throttle > self.accel_limit:
            throttle = self.accel_limit
            
         # Apply Correction for Yaw Steering
        steering = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)

        return throttle, brake, steering

    def check_dbw_status(self, dbw_enabled):
        if (not self.current_dbw_enabled) and dbw_enabled:
            self.reset()
        else:
            self.current_dbw_enabled = False

    def reset(self):
        self.current_dbw_enabled = True
        self.linear_pid.reset()
        self.previous_time = None
