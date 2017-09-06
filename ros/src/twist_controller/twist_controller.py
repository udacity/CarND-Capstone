from yaw_controller import YawController
from pid import PID
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, kp, ki, kd, min_lim, max_lim, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self._pid_controller = PID(kp,kd,ki, min_lim, max_lim)
        self._yaw_controller = YawController(wheel_base,steer_ratio,min_speed,max_lat_accel,max_steer_angle)
        pass

    def control(self, plv, pav, clv, dbw_enabled, dt):
        throttle = 0
        brake = 0
        steer = 0
        if(dbw_enabled == True):
            throttle = self._pid_controller.step(plv - clv, dt)
            if(throttle < 0):
                brake = -1 * throttle
                throttle = 0
            steer = self._yaw_controller.get_steering(plv, pav, clv)

        else:
            self._pid_controller.reset()
            throttle = 0
            brake = 0
            steer = 0

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        rospy.loginfo("Throttle = " + str(throttle))
        rospy.loginfo("Brake = " + str(brake))
        rospy.loginfo("DBW Enabled = " + str(dbw_enabled))
        return throttle, brake, steer
