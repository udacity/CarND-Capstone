from yaw_controller import YawController
from pid import PID
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, kp, ki, kd, min_lim, max_lim, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self._pid_controller_throttle = PID(kp,ki,kd, min_lim, max_lim)
        self._pid_controller_steer    = PID(15,0.04,0.9, -1, 1)
        self._yaw_controller = YawController(wheel_base,steer_ratio,min_speed,max_lat_accel,max_steer_angle)
        pass

    def control(self, plv, pav, clv, dbw_enabled, dt):
        throttle = 0
        brake = 0
        steer = 0
        if(dbw_enabled == True):
            throttle = self._pid_controller_throttle.step(plv - clv, dt)
            if(throttle < 0):
                brake = -1 * throttle
                throttle = 0
            steer_err = self._yaw_controller.get_steering(plv, pav, clv)
            steer =  self._pid_controller_steer.step(steer_err, dt)
            rospy.loginfo("plv=%f, clv=%f, pav=%f ",throttle,clv, pav)
            rospy.loginfo("steer_err = %f, steer = %f", steer_err, steer)
            #steer = steer_err


        else:
            self._pid_controller_throttle.reset()
            throttle = 0
            brake = 0
            steer = 0

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        rospy.loginfo("Throttle = " + str(throttle))
        rospy.loginfo("Brake = " + str(brake))
        rospy.loginfo("DBW Enabled = " + str(dbw_enabled))
        return throttle, brake, steer
