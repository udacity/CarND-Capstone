#!/usr/bin/env python
import rospy
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_SPEED = 40.0


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement     
        control(*args, **kwargs)   
        pass

    def control(self, *args, **kwargs):
    #def control(self):
        # TODO: Change the arg, kwarg list to suit your needs

        velocity = rospy.Publisher('/current_velocity', double, queue_size=255)
        print('velocity: ',velocity)
        if (velocity < MAX_SPEED):
            velocity = velocity + ONE_MPH        
	print('new velocity: ',velocity)
        # Return throttle, brake, steer
        return velocity, 0., 0.

