# -*- coding: utf-8 -*-

import rospy
from pid import PID


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):

    
    def __init__(
        self,
        vehicle_mass,
        fuel_capacity,
        brake_deadband,
        decel_limit,
        accel_limit,
        wheel_radius,
        wheel_base,
        steer_ratio,
        max_lat_accel,
        max_steer_angle):
         
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        
        # TODO: tune parameters for PIDs
        # PID params previously defined in dbw_node for STEERING -> adapt to throttle/brake
        self.pid_velocity = PID(0.35, 0.01, 0.0,    # p, i, d
                                -5, 1.0)           # min, max
        
        # PID params previously defined in dbw_node
        self.pid_steering = PID(0.35, 0.01, 0.0,    # p, i, d
                                -1.0, 1.0)           # min, max


    def control(self, cte, v_current, v_target, elapsed_time):
        
        v_error = v_target - v_current
        
        throttle = 0.
        brake = 0.

        adapt_speed = self.pid_velocity.step(v_error, elapsed_time)
        if adapt_speed < 0. :
            # TODO: do exact calculation based on max deceleration,
            #   vehicle mass, wheel radius, ...
            brake = adapt_speed
        else:
            # TODO: do exact calculation based on ...
            throttle = adapt_speed

        steering = self.pid_steering.step(cte, elapsed_time)
        
        rospy.logwarn('cte=%.3f, v_error=%.3f, throttle=%.3f, brake=%.3f, steering=%.3f',
                      cte, v_error, throttle, brake, steering)
        
        return throttle, brake, steering
    
    
    def reset(self):

        self.pid_velocity.reset()
        self.pid_steering.reset()
