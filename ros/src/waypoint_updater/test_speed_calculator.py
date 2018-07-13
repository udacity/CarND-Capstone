import unittest
import numpy as np
from speed_calculator import SpeedCalculator
import matplotlib.pyplot as plt

class TestAcceleration(unittest.TestCase):
    def test_acceleration_distribution(self):
        """Test how the acceleration is distributed along an distance.

        Expected behavior:
           The speed shall change much more in the beginning than towards the end, when measuring the change per
           meter (not per second).
        """
        current_speed = 0.0
        target_speed = 30.0
        speed_calc = SpeedCalculator(target_speed=target_speed, current_speed=current_speed,
                                     target_acceleration=0.0, current_accleration=0.0,
                                     acceleration_limit=10.0, jerk_limit=10.0)
        distances = np.arange(0.0, 100.0, 1.0)
        speeds = [speed_calc.get_speed_at_distance(distance) for distance in distances]
        self.assertEqual(speeds[0], current_speed)
        self.assertEqual(speeds[-1], target_speed)

        # The speed should increase rapidly when going slow and then smooth out towards the target.
        previous_diff = np.inf
        diffs = []
        for speed1, speed2 in zip(speeds[:-1], speeds[1:]):
            diff = speed2 - speed1
            diffs.append(diff)
            self.assertLessEqual(diff, previous_diff)
            previous_diff = diff

#        plt.plot(speed_calc.distances, speed_calc.velocities)
#        plt.plot(distances, speeds, 'r')
#        plt.show()

    def test_jerk_limit(self):
        """Test how the jerk limit affects acceleration

        Expected behavior:
            The acceleration up to target speed shall be faster as the jerk_limit is increased.
        """
        current_speed = 0.0
        target_speed = 30.0
        previous_acceleration_distance = np.inf
        for jerk_limit in np.arange(5.0, 20.1, 2.5):
            speed_calc = SpeedCalculator(target_speed=target_speed, current_speed=current_speed,
                                         target_acceleration=0.0, current_accleration=0.0,
                                         acceleration_limit=np.inf, jerk_limit=jerk_limit)
            speeds = [speed_calc.get_speed_at_distance(distance) for distance in np.arange(0.0, 100.0, 1.0)]
            self.assertEqual(speeds[0], 0.0)
            self.assertEqual(speeds[-1], 30.0)
            # Check that the acceleration is faster when permitting more jerk.
            acceleration_distance = speeds.index(30.0)
            self.assertLess(acceleration_distance, previous_acceleration_distance)
            previous_acceleration_distance = acceleration_distance

    def test_acceleration_limit(self):
        """Test how the acceleration limit affects acceleration

        Expected behavior:
            The acceleration up to target speed shall be faster as the acceleration limit is increased, up to some
            point where the limitation no longer have any effect.
        """
        current_speed = 0.0
        target_speed = 30.0
        previous_acceleration_distance = np.inf
        is_acceleration_effectively_limited = True
        numFasterAccelerations = 0
        numEqualAccelerations = 0
        distances = np.arange(0.0, 100.0, 1.0)

        for acc_limit in np.arange(5.0, 25.1, 2.5):
            speed_calc = SpeedCalculator(target_speed=target_speed, current_speed=current_speed,
                                         target_acceleration=0.0, current_accleration=0.0,
                                         acceleration_limit=acc_limit, jerk_limit=10.0)
            speeds = [speed_calc.get_speed_at_distance(distance) for distance in distances]
            accs = [speed_calc.get_acceleration_at_distance(distance) for distance in distances]
            self.assertEqual(speeds[0], 0.0)
            self.assertEqual(speeds[-1], 30.0)
            self.assertTrue(all(accs <= acc_limit))


            acceleration_distance = speeds.index(30.0)
            if is_acceleration_effectively_limited:
                if acceleration_distance < previous_acceleration_distance:
                    numFasterAccelerations += 1
                else:
                    self.assertTrue(all(accs < acc_limit))
                    is_acceleration_effectively_limited = False
                    numEqualAccelerations += 1
            else:
                # Acceleration is no longer reaching the limit
                self.assertTrue(all(accs < acc_limit))
                self.assertEqual(acceleration_distance, previous_acceleration_distance)
                numEqualAccelerations += 1
            previous_acceleration_distance = acceleration_distance
        self.assertTrue(numFasterAccelerations > 0)
        self.assertTrue(numEqualAccelerations > 0)

    def test_deceleration_distribution(self):
        """Test how the deceleration is distributed along an distance.

        Expected behavior:
           The speed shall decrease less in the beginning than towards the end, when measuring the change per
           meter (not per second).
        """
        current_speed = 30.0
        target_speed = 0.0
        speed_calc = SpeedCalculator(target_speed=target_speed, current_speed=current_speed,
                                     target_acceleration=0.0, current_accleration=0.0,
                                     acceleration_limit=10.0, jerk_limit=10.0)

        distances = np.arange(0.0, 100.0, 1.0)
        speeds = [speed_calc.get_speed_at_distance(distance) for distance in distances]
        self.assertEqual(speeds[0], current_speed)
        self.assertEqual(speeds[-1], target_speed)
        distances = distances[:speeds.index(target_speed)]
        speeds = speeds[:speeds.index(target_speed)]
        # The speed should increase rapidly when going slow and then smooth out towards the target.
        previous_diff = np.inf
        for speed1, speed2 in zip(speeds[:-1], speeds[1:]):
            diff = speed2 - speed1
            self.assertLessEqual(diff, previous_diff)
            previous_diff = diff

#        plt.plot(speed_calc.distances, speed_calc.velocities)
#        plt.plot(distances, speeds, 'r')
#        plt.show()


if __name__ == '__main__':
    unittest.main()
