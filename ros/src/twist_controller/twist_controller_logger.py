import time
import rospy
import numpy as np


class TwistControllerLogger:
    def __init__(self, twist_controller, rate):
        self.twist_controller = twist_controller
        self.logging_interval = 1000 / rate
        self.last_time_logged = self.current_time()

    def log(self, steer, steer_filtered, yaw_steer):
        if not self.should_log():
            return

        self.last_time_logged = self.current_time()

        line1 = 'steer: {}, steer_filtered: {} (+{})'.format(
            steer,
            steer_filtered,
            yaw_steer
        )

        line0 = '--- dbw node - twist controller'
        line0 = line0 + ('-' * (np.max([len(line1)]) - len(line0)))

        rospy.loginfo('')
        rospy.loginfo(line0)
        rospy.loginfo(line1)
        rospy.loginfo('')

    def should_log(self):
        return self.current_time() - self.last_time_logged > self.logging_interval

    @staticmethod
    def current_time():
        return int(round(time.time() * 1000))
