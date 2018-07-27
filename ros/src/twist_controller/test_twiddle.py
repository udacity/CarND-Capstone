import unittest
from twiddle import Twiddle
from pid import PID
import random
import math


class TestTwiddle(unittest.TestCase):
    def test_twiddle_inactive(self):
        """Twiddle should behave just as the PID controller when inactive"""
        pid = PID(coeffs=[1.0, 2.0, 3.0])
        twiddle = Twiddle(coeffs=[1.0, 2.0, 3.0])
        for i in range(1000):
            error = random.uniform(-10.0, 10.0)
            self.assertEqual(twiddle.step(error, 1), pid.step(error, 1))

    def test_twiddle_improves_pid_controller(self):
        twiddle = Twiddle(coeffs=[0.01, 0.0001, 0.03], active=True)
        lowest_error = float('inf')
        verify_runs = [0, 500, 1000]
        sys = TestSystem()

        for i in range(1001):
            if i not in verify_runs:
                sys.run(twiddle)
                twiddle.set_next_params()
            else:
                twiddle.abort()
                error = sys.run(twiddle)
                self.assertLess(error, lowest_error)
                lowest_error = error
                twiddle.cont()


class TestSystem(object):
    """Simple test system to help verify that the twiddle algorithm improves the PID controller"""
    def __init__(self):
        self.reset()

    def reset(self):
        """Reset system state"""
        self.__hidden = 0.0
        self.__target = 0.0
        self.__actual = 0.0

    def update(self, control_signal):
        """Update system state"""
        self.__hidden += 0.1
        self.__target = math.sin(self.__hidden)
        self.__actual += control_signal
        return self.__actual - self.__target

    def run(self, pid):
        """Run a number of update steps and accumulate the error"""
        self.reset()
        accumulated_error = 0.0
        control_signal = 0.0

        for i in range(100):
            error = self.update(control_signal)
            accumulated_error += abs(error)
            control_signal = pid.step(error, 1.0)

        return accumulated_error


if __name__ == '__main__':
    unittest.main()
