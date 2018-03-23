import rospy
from math import atan

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehMass, accLim, decLim, wheelBase, steerRatio, maxSteer):
        self.vehMass = vehMass
        self.accLim = accLim
        self.decLim = decLim
        self.wheelBase = wheelBase
        self.steerRatio = steerRatio
        self.maxSteer = maxSteer

        pass

    def control(self, curr_vel, target_vel, yaw):
        # TODO: Change the arg, kwarg list to suit your needs

        # Very stupid implementation as a first step
        rospy.loginfo('Current vel is %s, target vel is %s', curr_vel, target_vel)
        err = target_vel-curr_vel
        if err > 0:
            throttle = 1. * err
            brake = 0.
        else:
            throttle = 0.
            brake = - 0.1 * err

        steer = atan(self.wheelBase / (curr_vel/yaw)) * self.steerRatio

        # Return throttle, brake, steer
        return throttle, brake, steer
