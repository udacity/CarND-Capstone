import time
import rospy
import numpy as np


class DBWLogger:
    def __init__(self, dbw, rate):
        self.dbw = dbw
        self.logging_interval = 1000 / rate
        self.last_time_logged = self.current_time()

    def log(self, throttle, brake, steer, yaw_steer, cte):
        if not self.should_log():
            return

        self.last_time_logged = self.current_time()

        # todo: log self.final_waypoints = None
        # todo: log throttle
        # todo: log brake

        line1 = 'dbw: {}; linear v: {}; target linear v: {}; target angular v: {}'.format(
            self.dbw.dbw_enabled,
            self.dbw.current_linear_velocity,
            self.dbw.target_linear_velocity,
            self.dbw.target_angular_velocity
        )

        line2 = 'car_xy: ({}, {}); cte: {}; steer: {}'.format(
            'None' if self.dbw.current_pose is None else self.dbw.current_pose.position.x,
            'None' if self.dbw.current_pose is None else self.dbw.current_pose.position.y,
            cte,
            steer
        )

        line0 = '--- dbw node '
        line0 = line0 + ('-' * (np.max([len(line1), len(line2)]) - len(line0)))

        rospy.loginfo('')
        rospy.loginfo(line0)
        rospy.loginfo(line1)
        rospy.loginfo(line2)
        rospy.loginfo('')

    def should_log(self):
        return self.current_time() - self.last_time_logged > self.logging_interval

    @staticmethod
    def current_time():
        return int(round(time.time() * 1000))
