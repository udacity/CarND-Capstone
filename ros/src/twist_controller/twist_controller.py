
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
<<<<<<< Updated upstream
        # TODO: Implement
        pass
=======
        self.throttle_pid = pid.PID(THROTTLE_P_VAL, THROTTLE_I_VAL,
                                    THROTTLE_D_VAL, mn=MIN_LINEAR_VELOCITY,
                                    mx=MAX_LINEAR_VELOCITY)
        self.steering_pid = pid.PID(STEER_P_VAL, STEER_I_VAL, STEER_D_VAL,
                                    mn=MIN_ANGULAR_VELOCITY, mx=MAX_ANGULAR_VELOCITY)

    def control(self, target_linear_velocity, target_angular_velocity, 
                current_linear_velocity, current_angular_velocity,
                dbw_status, sample_time, **kwargs):
>>>>>>> Stashed changes

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
