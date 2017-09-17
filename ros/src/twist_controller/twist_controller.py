
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        pass

    # note here we don't care about the steering. We only control the throttle
    def control(self, prop_linear_vel, prop_angular_vel, cur_linear_vel, cur_angular_vel, dbw_status):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        return 1., 0., 0.
