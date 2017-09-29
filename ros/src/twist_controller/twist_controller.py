from pid import PID


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self):
        # TODO: Implement
        kp = 0.5
        ki = 0
        kd = 0
        self.speed_pid = PID(kp, ki, kd)
        pass

    def control(self, error):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        throttle = self.speed_pid.step(error, 0.1)
        return throttle, 0., 0.
