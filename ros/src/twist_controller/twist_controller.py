import rospy
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    """ Twist controller to predict throttle, brake, and steer."""
    def __init__(self, *args, **kwargs):
        self.decel_limit = kwargs.get('decel_limit')
        self.accel_limit = kwargs.get('accel_limit')
        self.max_steer_angle = kwargs.get('max_steer_angle')

        # Pid
        self.throttle_pid = PID(kp=0.0, ki=0.0, kd=0.0, mn=self.decel_limit, mx=self.accel_limit)
        self.steer_pid = PID(kp=0.0, ki=0.0, kd=0.0, mn=-self.max_steer_angle, mx=self.max_steer_angle)

        self.timestamp = rospy.get_time()

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
