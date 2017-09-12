import numpy as np
import rospy
import tf
import math
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    """ Twist controller to predict throttle, brake, and steer."""
    def __init__(self, *args, **kwargs):
        self.decel_limit = kwargs.get('decel_limit')
        self.accel_limit = kwargs.get('accel_limit')
        self.max_steer_angle = kwargs.get('max_steer_angle')

        # Pid
        self.throttle_pid = PID(kp=0.0, ki=0.0, kd=0.0, mn=self.decel_limit, mx=self.accel_limit)
        self.steer_pid = PID(kp=0.0, ki=0.0, kd=0.0, mn=-self.max_steer_angle, mx=self.max_steer_angle)

        self.cte = None
        self.diff_velocity = 0

        self.timestamp = None

    def control(self, *args, **kwargs):
        pose = kwargs.get('pose')
        waypoints = kwargs.get('waypoints')
        velocity = kwargs.get('velocity')
        target_velocity = kwargs.get('target_velocity')

        if self.timestamp is None:
            self.timestamp = rospy.get_time()

        current_timestamp = rospy.get_time()
        self.timestamp = current_timestamp
        sample_time = current_timestamp - self.timestamp

        cte = self.get_cte(pose, waypoints)
        steer = self.steer_pid.step(cte, sample_time)
        # TODO: Implement filters
        throttle = self.throttle_pid.step(target_velocity-velocity, sample_time)
        brake = 0

        return throttle, brake, steer

    def reset(self):
        """Reset the PIDs

        Reset all the settings when user changes from auto to manual driving
        """
        self.timestamp = None
        self.throttle_pid.reset()
        self.steer_pid.reset()

    def get_cte(self, pose, waypoints):
        x_coords, y_coords = [], []
        # Current car pose and head (used as waypoint 0)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w])
        heading_x = pose.position.x
        heading_y = pose.position.y

        for waypoint in waypoints:
            # Calculate waypoint wrt current car position
            waypoint_x = waypoint.pose.pose.position.x - heading_x
            waypoint_y = waypoint.pose.pose.position.y - heading_y
            x = waypoint_x * math.cos(0 - yaw) - waypoint_y * math.sin(0 - yaw)
            y = waypoint_y * math.sin(0 - yaw) - waypoint_x * math.cos(0 - yaw)

            x_coords.append(x)
            y_coords.append(y)

            coeffs = np.polyfit(x_coords, y_coords, 3)
            self.cte = coeffs[0]
