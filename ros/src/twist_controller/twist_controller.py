import rospy
import os
import pickle
import copy
from lowpass import LowPassFilter
from pid import PID
from std_msgs.msg import Int32, Float32


from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        vehicle_mass = kwargs['vehicle_mass']
        fuel_capacity = kwargs['fuel_capacity']
        wheel_radius = kwargs['wheel_radius']
        wheel_base = kwargs['wheel_base']
        steer_ratio = kwargs['steer_ratio']
        max_lat_accel = kwargs['max_lat_accel']
        max_steer_angle = kwargs['max_steer_angle']
        self.torque = (vehicle_mass + fuel_capacity*GAS_DENSITY) * wheel_radius
        min_speed = 0.001

        self.velocity_pid = PID(kp=0.1, ki=0.0001, kd=0.1, mn=self.decel_limit, mx=self.accel_limit)
        self.steer_pid = PID(kp=0.5, ki=0.01, kd=0.1, mn=-max_steer_angle, mx=max_steer_angle)
        #self.steer_PID = PID(0.2, 0.0000001, 0.5, mn = -self.max_steer_angle, mx = self.max_steer_angle) # To be adjusted
        self.lowpass = LowPassFilter(0.3, 0.3)
        self.steer_filter = LowPassFilter(0.4, 0.2)
        
        params = [wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle]
        self.yaw_controller = YawController(*params)

        self.brake_torque = (vehicle_mass + fuel_capacity * GAS_DENSITY) * wheel_radius
         
        # Get the laste timestamp
        self.last_time = rospy.get_time()
        self.dbw_enabled = False

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        # linear_setpoints 
        tgt_linear_setpoint = kwargs['linear_setpoint']
        tgt_angular_setpoint = kwargs['angular_setpoint']
        cur_linear_current = kwargs['linear_current']
        
        
        # Target and current 
       
        velocity_error = tgt_linear_setpoint - cur_linear_current
        
        # Calculate he delta t
        curr_time = rospy.get_time()
        delta = curr_time - self.last_time if self.last_time else 0.1
        #delta = 0.02
        self.last_time = curr_time
        
        # PID set for the speed 
        # PID set for the speed  and use low pass to filter the result for rhe speed 
        velocity = self.lowpass.filt(self.velocity_pid.step(velocity_error, delta))
        
        steer_raw = self.yaw_controller.get_steering(tgt_linear_setpoint, tgt_angular_setpoint, cur_linear_current)
        steer = self.steer_pid.step(steer_raw, delta)
        steer = self.steer_filter.filt(steer_raw)
        

        
        

        throttle = 0.
        brake = 0.

        # if setpoint velocity less than threshold do max brake (fixes low throttle crawl at light)
        if tgt_linear_setpoint < 0.01:
            steer = 0. # yaw controller has trouble at low speed
            brake = abs(self.decel_limit) * self.brake_torque
        else:
            # speeding up - set velocity to throttle 
            if velocity > 0.:
                throttle = velocity
            # slowing down - check deadband limit before setting brake
            else:
                velocity = abs(velocity)

                # brake if outside deadband
                if velocity > self.brake_deadband:
                    brake = velocity * self.brake_torque

        
        return throttle, brake, steer