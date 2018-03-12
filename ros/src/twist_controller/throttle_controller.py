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

    def __init__(self):
        self.throttle_pid = PID(kp=1.0, ki=1.0, kd=1.0)

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

        throttle = self.throttle_pid.step(error, dt)
        throttle, brake = max(0.0, min(1.0, throttle)), 0.0

        return throttle, brake
