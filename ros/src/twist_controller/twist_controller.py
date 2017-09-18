from datetime import datetime

from pid import PID


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement

        self.pid = PID(1., 0, 0, 0, 1)
        self.prev_timestamp = datetime.now()


    def control(self, proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity):

        timestamp = datetime.now()

        elapsed_time = timestamp - self.prev_timestamp

        error = proposed_linear_velocity - current_linear_velocity

        throttle = self.pid.step(error, elapsed_time.total_seconds())
        print("throttle:{}".format(throttle))

        # Return throttle, brake, steer
        return throttle, 0., proposed_angular_velocity * 10
