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
            # No acceleration. Current speed will be returned regardless of distance.
            self.distance_speed_lut = [current_speed]

        # The list of distances represented in the LUT
        self.distances = [i * self.delta_d for i in range(len(self.distance_speed_lut))]

    def __accelerate(self, target_speed, current_speed,
                     target_acceleration, current_accleration,
                     acceleration_limit, jerk_limit):
        # Calculate the velocity at each time-step when applying acceleration and jerk according to their max limits.
        # Instead of continuously calculate when the jerk must change sign such that the acceleration can be reduced to
        # zero at target speed, the iteration is started from both ends to find the speed overlap in the middle.
        v0 = current_speed
        a0 = current_accleration
        v1 = target_speed
        a1 = target_acceleration
        j0 = jerk_limit
        j1 = -jerk_limit
        delta_t = 0.001

        velocity_from_start_time = []
        velocity_from_end_time = []

        while v1 > v0:
            velocity_from_start_time.append(v0)
            velocity_from_end_time.append(v1)
            v0 += a0 * delta_t
            v1 -= a1 * delta_t
            a0 = min(acceleration_limit, a0 + j0 * delta_t)
            a1 = min(acceleration_limit, a1 - j1 * delta_t)

        # Calculate if the approximation is better with or without the middle point.
        if (v0 - v1) < (a0 / 2.0):
            velocity_from_start_time.append((v0 + v1) / 2.0)

        # Concatenate the both lists into one common list from the start.
        velocity_from_start_time += reversed(velocity_from_end_time)

        # Calculate the distance at each time-step.
        distance_from_start_time = []
        d = 0.0
        for v in velocity_from_start_time:
            d += v * delta_t
            distance_from_start_time.append(d)

        # Calculate a look-up table for the velocity at a given distance.
        self.distance_speed_lut = []
        d = 0.0
        for i, d1 in enumerate(distance_from_start_time):
            while d <= d1:
                self.distance_speed_lut.append(velocity_from_start_time[i])
                d += self.delta_d
        # Make sure target velocity not is omitted (due to resolution of delta_d)
        self.distance_speed_lut.append(velocity_from_start_time[-1])

    def __decelerate(self, target_speed, current_speed,
                     target_acceleration, current_accleration,
                     acceleration_limit, jerk_limit):
        # Swap current and target speed/accelerations and then use the accelerate method
        self.__accelerate(target_speed=current_speed, current_speed=target_speed,
                          target_acceleration=current_accleration, current_accleration=target_acceleration,
                          acceleration_limit=acceleration_limit, jerk_limit=jerk_limit)
        self.distance_speed_lut.reverse()

    def get_speed_at_distance(self, distance):
        return np.interp(distance, self.distances, self.distance_speed_lut)
