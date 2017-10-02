import rospy
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED = 0.
ZERO_VALUES = [0., 0., 0.]

class Controller(object):
    def __init__(self, **kwargs):
        print("Keyword arguments:", kwargs)
        # TODO: Implement
        self.time = rospy.get_time()

        self.pid = PID(kp=0.5,
                       ki=0.003,
                       kd=0.25,
                       mn=-kwargs['max_steer_angle'],
                       mx=kwargs['max_steer_angle'])

        self.yaw = YawController(wheel_base=kwargs['wheel_base'],
                                 steer_ratio=kwargs['steer_ratio'],
                                 min_speed=MIN_SPEED,
                                 max_lat_accel=kwargs['max_lat_accel'],
                                 max_steer_angle=kwargs['max_steer_angle'])

    def control(self, twist_cmd, current_velocity, dbw_enabled):
        # print('\nt:\n{0}, \nv:\n{1}, \nd:{2}'.format(twist_cmd, current_velocity, dbw_enabled))

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        _time = rospy.get_time()
        sample_time = _time - self.time
        self.time = _time

        if not all((twist_cmd, current_velocity)):
            return ZERO_VALUES

        if dbw_enabled == True:
            throttle = brake = 0.
            control = self.pid.step(twist_cmd.twist.linear.x - current_velocity.twist.linear.x, sample_time)
            print("c:{0}".format(control))

            if control >= 0.:
                throttle = control
            else:
                brake = control
            # TODO linear_velocity, and angular_velocity
            steering = self.yaw.get_steering(twist_cmd.twist.linear.x,
                                             twist_cmd.twist.angular.z,
                                             current_velocity.twist.linear.x)
            return throttle, brake, steering

        self.pid.reset()
        return ZERO_VALUES
