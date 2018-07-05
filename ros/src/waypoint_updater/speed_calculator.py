import numpy as np


class SpeedCalculator(object):
    """Calculates the vehicle speed at specific positions during an acceleration"""
    def __init__(self, target_speed, current_speed=0.0,
                 target_acceleration=0.0, current_accleration=0.0,
                 acceleration_limit=10.0, jerk_limit=10.0):

        self.delta_d = 0.1  # Resolution of LUT
        if target_speed > current_speed:
            self.__accelerate(target_speed, current_speed,
                              target_acceleration, current_accleration,
                              acceleration_limit, jerk_limit)
        elif target_speed < current_speed:
            self.__decelerate(target_speed, current_speed,
                              target_acceleration, current_accleration,
                              acceleration_limit, jerk_limit)
        else:
            # No acceleration. Current speed/acceleration will be returned regardless of distance.
            self.distances = [0.0]
            self.velocities = [current_speed]
            self.accelerations = [current_accleration]

    def __accelerate(self, target_speed, current_speed,
                     target_acceleration, current_accleration,
                     acceleration_limit, jerk_limit):
        # Calculate the velocity at each time-step when applying acceleration and jerk according to their max limits.
        # Instead of continuously calculate when the jerk must change sign such that the acceleration can be reduced to
        # zero at target speed, the iteration is started from both ends to find the speed overlap in the middle.
        delta_t = 0.01
        vs_current = [current_speed]
        vs_target = [target_speed]
        as_current = [current_accleration]
        as_target = [target_acceleration]

        while vs_current[-1] < vs_target[-1]:
            vs_current.append(vs_current[-1] + as_current[-1] * delta_t)
            vs_target.append(vs_target[-1] - as_target[-1] * delta_t)
            as_current.append(min(acceleration_limit, as_current[-1] + jerk_limit * delta_t))
            as_target.append(min(acceleration_limit, as_target[-1] + jerk_limit * delta_t))

        # Calculate the middle time-step
        middle_delta_t = (vs_target[-1] - vs_current[-1]) / as_current[-1]
        delta_ts = [delta_t] * len(vs_current) + [middle_delta_t] + [delta_t] * len(vs_target)

        # Concatenate the list starting from current with the reversed one starting from target.
        self.velocities = vs_current + vs_target[::-1]
        self.accelerations = as_current + as_target[::-1]

        # Calculate the distance at each time-step.
        self.distances = []
        d = 0.0
        for delta_t, v in zip(delta_ts, self.velocities):
            d += v * delta_t
            self.distances.append(d)

    def __decelerate(self, target_speed, current_speed,
                     target_acceleration, current_accleration,
                     acceleration_limit, jerk_limit):
        # Swap current and target speed/accelerations and then use the accelerate method
        self.__accelerate(target_speed=current_speed, current_speed=target_speed,
                          target_acceleration=current_accleration, current_accleration=target_acceleration,
                          acceleration_limit=acceleration_limit, jerk_limit=jerk_limit)
        self.velocities.reverse()
        self.accelerations.reverse()
        self.distances = [self.distances[-1] - distance for distance in self.distances[::-1]]

    def get_speed_at_distance(self, distance):
        return np.interp(distance, self.distances, self.velocities)

    def get_acceleration_at_distance(self, distance):
        return np.interp(distance, self.distances, self.accelerations)
