"""
Author: Peng Xu <robotpengxu@gmail.com>
Date:   March 10, 2018
"""

'''
A brake controller based on torque for dbw node
'''
GAS_DENSITY = 2.858


class BrakeController(object):
    """ A Speed Controller class """

    def __init__(self, vehicle_mass, wheel_radius, decel_limit, brake_deadband, fuel_capacity):
        self.vehicle_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.wheel_radius = wheel_radius
        self.decel_limit = decel_limit
        self.brake_deadband = brake_deadband
        # max brake torque corresponding to deceleration limit
        self.max_torque = self.vehicle_mass * abs(self.decel_limit) * self.wheel_radius

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

        accel = error / dt
        accel = max(self.decel_limit, accel)

        throttle, brake = 0., 0.
        # no controls if we are in the deadband
        if abs(accel) < self.brake_deadband:
            return throttle, brake

        # calculate torque = M*acc*R
        torque = self.vehicle_mass * accel * self.wheel_radius

        throttle, brake = 0.0, min(abs(torque), self.max_torque)

        return throttle, brake
