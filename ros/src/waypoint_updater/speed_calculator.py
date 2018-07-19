import numpy as np


class SpeedCalculator(object):
    """Calculates the vehicle speed at specific positions during an acceleration"""
    def __init__(self, target_speed, current_speed=0.0,
                 target_acceleration=0.0, current_accleration=0.0,
                 acceleration_limit=10.0, jerk_limit=10.0, delta_t=0.01):

        self.acceleration_limit = acceleration_limit
        self.jerk_limit = jerk_limit
        self.delta_t = delta_t

        if target_speed > current_speed:
            self.__accelerate(target_speed, current_speed,
                              target_acceleration, current_accleration)
        elif target_speed < current_speed:
            self.__decelerate(target_speed, current_speed,
                              target_acceleration, current_accleration)
        else:
            # No acceleration. Current speed/acceleration will be returned regardless of distance.
            self.distances = [0.0]
            self.velocities = [current_speed]
            self.accelerations = [current_accleration]

    def __accelerate(self, target_speed, current_speed,
                     target_acceleration, current_accleration):
        # Calculate the velocity at each time-step when applying acceleration and jerk according to their max limits.
        # Instead of continuously calculate when the jerk must change sign such that the acceleration can be reduced to
        # zero at target speed, the iteration is started from both ends to find the speed overlap in the middle.
        vs_current = [current_speed]
        vs_target = [target_speed]
        as_current = [current_accleration]
        as_target = [target_acceleration]

        # Make sure the current and target acceleration is balanced
        if not np.isclose(current_accleration, target_acceleration):
            self.__balance_up_acceleration(vs_current, vs_target, as_current, as_target)

        # Calculate the the speeds and accelerations from both ends until reaching the overlap.
        while vs_current[-1] < vs_target[-1]:
            vs_current.append(vs_current[-1] + as_current[-1] * self.delta_t)
            vs_target.append(vs_target[-1] - as_target[-1] * self.delta_t)
            as_current.append(min(self.acceleration_limit, as_current[-1] + self.jerk_limit * self.delta_t))
            as_target.append(min(self.acceleration_limit, as_target[-1] + self.jerk_limit * self.delta_t))

        # Calculate the middle time-step
        middle_delta_t = (vs_target[-1] - vs_current[-1]) / as_current[-1]
        delta_ts = [self.delta_t] * len(vs_current) + [middle_delta_t] + [self.delta_t] * len(vs_target)

        # Concatenate the list starting from current with the reversed one starting from target.
        self.velocities = vs_current + vs_target[::-1]
        self.accelerations = as_current + as_target[::-1]

        # Calculate the distance at each time-step.
        self.distances = []
        d = 0.0
        for delta_t, v in zip(delta_ts, self.velocities):
            d += v * delta_t
            self.distances.append(d)

    def __balance_up_acceleration(self, vs_current, vs_target, as_current, as_target):
        if as_current[-1] < as_target[-1]:
            while as_current[-1] < as_target[-1] and vs_current[-1] < vs_target[-1]:
                vs_current.append(vs_current[-1] + as_current[-1] * self.delta_t)
                as_current.append(min(self.acceleration_limit, as_current[-1] + self.jerk_limit * self.delta_t))
        elif as_current[-1] > as_target[-1]:
            while as_current[-1] > as_target[-1] and vs_current[-1] < vs_target[-1]:
                vs_target.append(vs_target[-1] - as_target[-1] * self.delta_t)
                as_target.append(min(self.acceleration_limit, as_target[-1] + self.jerk_limit * self.delta_t))

    def __decelerate(self, target_speed, current_speed,
                     target_acceleration, current_accleration):
        # Swap current and target speed/accelerations and then use the accelerate method
        self.__accelerate(target_speed=current_speed, current_speed=target_speed,
                          target_acceleration=current_accleration, current_accleration=target_acceleration)
        self.velocities.reverse()
        self.accelerations.reverse()
        self.distances = [self.distances[-1] - distance for distance in self.distances[::-1]]

    def get_speed_at_distance(self, distance):
        return np.interp(distance, self.distances, self.velocities)

    def get_acceleration_at_distance(self, distance):
        return np.interp(distance, self.distances, self.accelerations)
