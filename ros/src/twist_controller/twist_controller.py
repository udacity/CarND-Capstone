import numpy as np
import rospy
import tf
import math
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    """ Twist controller to predict throttle, brake, and steer."""
    def __init__(self, *args, **kwargs):
        self.decel_limit = kwargs.get('decel_limit')
        self.accel_limit = kwargs.get('accel_limit')
        self.max_steer_angle = kwargs.get('max_steer_angle')

        # Vehicle mass, fuel mass and wheel radius are required to calculate braking torque
        self.vehicle_mass = kwargs.get('vehicle_mass')
        self.wheel_radius = kwargs.get('wheel_radius')
        self.fuel_mass = kwargs.get('fuel_capacity') * GAS_DENSITY # Assuming a full tank of gas

        # A band of deceleration within which brakes don't apply (because it's too low)
        self.brake_deadband = kwargs.get('brake_deadband')

        # Controllers
        self.throttle_pid = PID(kp=1, ki=0.005, kd=0.5, mn=self.decel_limit, mx=self.accel_limit)
        self.yaw_controller = YawController(
            wheel_base=kwargs.get('wheel_base'),
            steer_ratio=kwargs.get('steer_ratio')*8,
            min_speed=kwargs.get('min_speed'),
            max_lat_accel=kwargs.get('max_lat_accel'),
            max_steer_angle=kwargs.get('max_steer_angle')
        )
        self.low_pass_filter = LowPassFilter(0.2, 0.1)

        self.timestamp = None

    def control(self, *args, **kwargs):
        # For steering, we need the vehicle velocity (angular and linear)
        # For velocity, we need the current velocity and target velocity to calculate the error
        current_velocity = kwargs.get('current_velocity')
        linear_velocity = kwargs.get('linear_velocity') # or target velocity
        angular_velocity = kwargs.get('angular_velocity')

        throttle, brake, steer = 0., 0., 0.

        # If this is the first time control() is called
        if self.timestamp is None:
            self.timestamp = rospy.get_time()
            return throttle, brake, steer

        current_timestamp = rospy.get_time()
        delta_time = current_timestamp - self.timestamp
        timestamp = current_timestamp

        # current_velocity = current_velocity / ONE_MPH # Too slow
        velocity_error = linear_velocity - current_velocity

        # Make sure we have a valid delta_time. We don't want to divide by zero.
        # Since we're publishing at 50Hz, the expected delta_time should be around 0.02
        if delta_time > 0.01:
            acceleration = self.throttle_pid.step(velocity_error, delta_time)

            # Throttle when acceleration is positive
            # Brake when acceleration is negative
            if acceleration >= 0:
                throttle = acceleration
                brake = 0.
            else:
                # Brake calculations (Brake value is in Torque (N * m))
                # (i.e. how much torque would we need to reduce our acceleration by x?)

                # Braking Torque = Force * Distance from point of rotation
                # * Distance from point of rotation = Wheel radius
                # * Force = Mass * target deceleration
                # * Mass = Vehicle mass + Fuel mass (assuming full tank)

                # Only apply brakes if the required deceleration is significant enough
                # (Greater than the specified brake_deadband)
                deceleration = abs(acceleration)
                if deceleration > self.brake_deadband:
                    brake = (self.vehicle_mass + self.fuel_mass) * deceleration * self.wheel_radius
                else:
                    brake = 0.

                throttle = 0.

        # throttle = self.throttle_pid.step(velocity_error, delta_time)
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        # Apply low pass filters to the throttle and brake values to eliminate jitter
        throttle = self.low_pass_filter.filt(throttle)
        brake = self.low_pass_filter.filt(0.0)

        return throttle, brake, steer

    def reset(self):
        """Reset the PIDs

        Reset all the settings when user changes from auto to manual driving
        """
        self.timestamp = None
        self.throttle_pid.reset()
        # self.steer_pid.reset()

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
