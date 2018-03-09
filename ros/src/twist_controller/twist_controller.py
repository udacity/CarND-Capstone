from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self, accel_limit, decel_limit, yaw_dot_limit):
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.yaw_dot_limit = yaw_dot_limit

        self.accel_pid = PID(0.40,   # kp
                             0.00,   # ki
                             0.09,   # kd
                             self.decel_limit,
                             self.accel_limit)

        self.yaw_dot_pid = PID(0.40,    # kp
                               0.00,    # ki
                               0.09,    # kd
                               0-yaw_dot_limit,
                               yaw_dot_limit)

    def control(self, target_accel, target_yaw_dot, dt):
        accel = self.accel_pid.step(target_accel, dt)
        yaw_dot = self.yaw_dot_pid.step(target_yaw_dot, dt)
        return accel, yaw_dot

    def reset(self):
        self.reset_accel_pid()
        self.reset_yaw_dot_pid()

    def reset_accel_pid(self):
        self.accel_pid.reset()

    def reset_yaw_dot_pid(self):
        self.yaw_dot_pid.reset()

