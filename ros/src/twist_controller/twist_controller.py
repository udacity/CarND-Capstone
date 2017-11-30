
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, steer_ratio):
        self.steer_ratio = steer_ratio

    def control(self, target_linear_velocity, target_angular_velocity,
                current_linear_velocity, dbw_status):
        '''Defines target throttle, brake and steering values'''

        # TODO: implement throttle controller
        throttle = 0.5

        # TODO: implement brake controller
        brake = 0

        # implement steering controller
        steering = self.steer_ratio * target_angular_velocity
        
        return throttle, brake, steering
