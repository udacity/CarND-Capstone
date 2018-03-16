"""
Author: Ravel Antunes <ravelantunes@gmail.com>
        Peng Xu <robotpengxu@gmail.com>
Date:   March 10, 2018
"""

'''
A throttle pid controller based for dbw node
'''

from pid import PID


class ThrottleController(object):
    """ A Speed Controller class """

    def __init__(self, decel_limit, accel_limit):
        self.throttle_pid = PID(kp=1.5, ki=0.001, kd=0.0, mn=decel_limit, mx=accel_limit)

    def control(self, error, dt):
        """
        Given the velocity error and the time elapsed
        since previous step it returns the correct throttle brake combination

        Args:
             error (float) : the velocity error of the car (target velocity - current velocity)
             dt (float): The time to make the transition

        Returns:
             throttle (float) , brake (float)
        """

        self.throttle_pid.reset()
        throttle = self.throttle_pid.step(error, dt)
        throttle, brake = max(0.0, min(1.0, throttle)), 0.0

        return throttle, brake
