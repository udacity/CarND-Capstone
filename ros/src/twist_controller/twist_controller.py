from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import math

class Controller(object):

    def __init__(self, *args, **kwargs):
        self.speed_controller = PID(2, 0.005, 0.5)
        self.steering_controller = PID(2, 0.003, 1)

    def control(self, target_velocity, current_velocity, dbw_enabled, dt):
        target_linear_velocity, target_angular_velocity = target_velocity
        current_linear_velocity, current_angular_velocity = current_velocity

        # `rostopic echo /twist_cmd` says target linear velocity is fixed to 11.
        # but target angular velocity is changing based on vehicle's orientation.
        linear_velocity_cte = target_linear_velocity - current_linear_velocity
        angular_velocity_cte = target_angular_velocity

        if not dbw_enabled: # manual driving
            self.speed_controller.reset()
            self.steering_controller.reset()

        linear_velocity = self.speed_controller.step(linear_velocity_cte, dt)
        steering = self.steering_controller.step(angular_velocity_cte, dt)

        throttle = 1
        brake = 0

        if linear_velocity > 0:
            throttle = linear_velocity
        else:
            brake = linear_velocity

        return throttle, brake, steering