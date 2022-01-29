"""
This module implements a basic PID controller.
"""

MIN_NUM = float("-inf")
MAX_NUM = float("inf")


class PID:
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._min = mn
        self._max = mx

        self._int_val = self._last_error = 0.0

    def reset(self):
        self._int_val = 0.0

    def step(self, error, sample_time):

        integral = self._int_val + error * sample_time
        derivative = (error - self._last_error) / sample_time

        val = self._kp * error + self._ki * integral + self._kd * derivative

        if val > self._max:
            val = self._max
        elif val < self._min:
            val = self._min
        else:
            self._int_val = integral
        self._last_error = error

        return val
