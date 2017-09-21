from datetime import datetime

from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self, yaw_controller, max_steer_angle):
        # TODO: Implement
        
        self.yaw_controller = yaw_controller

        self.throttle_pid = PID(kp=1., ki=0, kd=0, mn=0, mx=1.)
        self.steering_pid = PID(kp=.04, ki=.004, kd=.4, mn=-max_steer_angle, mx=max_steer_angle)
        self.steering_pid = PID(kp=1, ki=0, kd=1, mn=-max_steer_angle, mx=max_steer_angle)


        self.steer_filter = LowPassFilter(tau=0.0, ts=1.0)

        self.prev_timestamp = datetime.now()

    def control(self, proposed_linear_velocity, proposed_angular_velocity,
                current_linear_velocity, current_angular_velocity):

        timestamp = datetime.now()
        elapsed_time = timestamp - self.prev_timestamp

        linear_cte = proposed_linear_velocity - current_linear_velocity

        angular_cte = proposed_angular_velocity - current_angular_velocity

        throttle = self.throttle_pid.step(linear_cte, elapsed_time.total_seconds())
        print("throttle:{}".format(throttle))

        steer = self.steering_pid.step(angular_cte, elapsed_time.total_seconds())
        print("steer:{}".format(steer))
        steer = self.steer_filter.filt(steer)
        print("filtered steer:{}".format(steer))

        steering = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity,
                                                    current_linear_velocity)
        print("steering:{}".format(steering))

        steer = steering + steer

        # Return throttle, brake, steer
        return throttle, 0., steer
