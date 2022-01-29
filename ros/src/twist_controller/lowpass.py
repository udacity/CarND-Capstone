#!/usr/bin/env python

"""
This module implements LowPassFilter
"""


class LowPassFilter:
    def __init__(self, tau, ts):
        self._a = 1.0 / (tau / ts + 1.0)
        self._b = tau / ts / (tau / ts + 1.0)

        self._last_val = 0.0
        self._ready = False

    def get(self):
        return self._last_val

    def filt(self, val):
        if self._ready:
            val = self._a * val + self._b * self._last_val
        else:
            self._ready = True

        self._last_val = val
        return val
