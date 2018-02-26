
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from yaw_controller import YawController
from pid import PID

class Controller(object):
    def __init__(self, *args, **kwargs):
        self.dbw_enabled = True
        # self.yaw_controller = YawController()
        self.throttle_pid_controller = PID(1.0, 1.0, 1.0)
        self.steering_pid_controller = PID(1.0, 1.0, 1.0)

        # Initialize state that will be updated nodes dbw is subscribed to
        self.current_velocity = None
        self.twist = None

    def toggle_dbw(self, dbw_enabled):
        self.dbw_enabled = dbw_enabled

    def control(self, linear, angular, current):

        

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
