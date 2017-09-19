from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        # specific for 25mph
        #self.steer_control_pid = PID(3.0,0.001,0.04,-kwargs['max_steer_angle'],kwargs['max_steer_angle'])
        self.speed_control_pid = PID(0.3,0.001,0.0,kwargs['decel_limit'],kwargs['accel_limit'])
        # Kp reduced from 3.0 to 2.0 to reduce jittering of car at 10mph but behaves better at higher speed(25mph)
        self.steer_control_pid = PID(2.0,0.001,0.04,-kwargs['max_steer_angle'],kwargs['max_steer_angle'])
        # Best for 25mph but not good for 10mph
        #self.steer_control_pid = PID(3.0,0.001,0.0,-kwargs['max_steer_angle'],kwargs['max_steer_angle'])
        self.min_speed = 0.0 #TBD
        self.steer_yaw_controller = YawController(kwargs['wheel_base'],kwargs['steer_ratio'],
                                    self.min_speed,kwargs['max_lat_accel'],
                                    kwargs['max_steer_angle'])
        self.decel_limit = kwargs['decel_limit']
        self.vehicle_mass = kwargs['vehicle_mass']
        self.wheel_radius = kwargs['wheel_radius']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.dbw_toggled = True
        self.previous_time = None

    def control(self, target_linear_vel, target_angular_vel, current_linear_vel,current_angular_vel,dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if( dbw_enabled) and (self.dbw_toggled):
           self.speed_control_pid.reset()
           self.steer_control_pid.reset()
           self.dbw_toggled = False
        elif(not dbw_enabled):
           self.dbw_toggled = True

        if dbw_enabled and self.previous_time is not None:

           # PID controller for throttle and brake 
           current_time = rospy.get_time()
           step_time = current_time - self.previous_time
           self.previous_time = current_time

           linear_vel_error = target_linear_vel - current_linear_vel
           self.vehicle_tot_mass = (self.vehicle_mass + GAS_DENSITY*self.fuel_capacity)
           speed_control = self.speed_control_pid.step(linear_vel_error, step_time)

           if linear_vel_error > 0:
              throttle = speed_control
              brake = 0.0
           else:
              brake = -speed_control
              if brake < self.brake_deadband and  brake >  0.:
                 brake = 0.0
              else:
                 brake =abs(speed_control * self.vehicle_tot_mass * self.wheel_radius)  # torque in N-m
              throttle = 0.0

           # YawController for steering calculation and PID controller for steer cmd
           desired_steer = self.steer_yaw_controller.get_steering(target_linear_vel,
                     target_angular_vel,current_linear_vel)
           actual_steer = self.steer_yaw_controller.get_steering(current_linear_vel,
                     current_angular_vel,current_linear_vel)
           steer = self.steer_control_pid.step((desired_steer - actual_steer),step_time)
           
           return throttle,brake,steer

        else:
           self.previous_time = rospy.get_time()
           return 0.,0.,0.

        

        
