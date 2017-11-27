from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import time
import rospy

GAS_DENSITY = 2.858
# ONE_MPH = 0.44704               # 1 miles/hour in meters/second

class TwistController(object):
    def __init__(self, wheel_base, vehicle_mass, steer_ratio, min_speed, max_lat_accel, max_steer_angle,
                 max_braking_percentage, max_throttle_percentage, max_vel_mps):
        # TODO: Implement
        self.sample_time = 0.03  # based on observation the interval is always around 0.03 seconds
        # self.throttle_controller = PID(2.0, 0.01, 0.02, mn=0.0, mx=1.0)
        self.yaw_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.lowpass_filter = LowPassFilter(0.15, self.sample_time)
        # change the past component coefficient from 0.5 to 0.15 to be more responsive
        self.brake_coefficient = 10.0  # tentative guess
        self.vehicle_mass = vehicle_mass
        self.prev_time = None
        self.steer_ratio = steer_ratio
        self.max_braking_percentage = max_braking_percentage
        self.max_throttle_percentage = max_throttle_percentage
        self.max_vel_mps = max_vel_mps

    def control(self, desired_linear_velocity, desired_angular_velocity,
                current_linear_velocity, current_angular_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        # if self.prev_time is None:
        #     self.prev_time = time.time()
        #     return 0., 0., 0.

        desired_linear_velocity_modulated = min(self.max_vel_mps, desired_linear_velocity)
        throttle, brake = 0.0, 0.0
        error_linear_velocity = (desired_linear_velocity_modulated - current_linear_velocity)
        # according to the forum:
        if desired_linear_velocity > current_linear_velocity:
            if error_linear_velocity / desired_linear_velocity > 0.3:
                throttle = self.max_throttle_percentage
            else:
                throttle = max(
                    (error_linear_velocity / desired_linear_velocity)/0.3*self.max_throttle_percentage,
                    self.max_throttle_percentage)
            # end of if error_linear_velocity / desired_linear_velocity > 0.3
        elif current_linear_velocity > 1:
            brake = 3250*self.max_braking_percentage*-1
        else:
            brake = 3250*0.01
        # end of if desired_linear_velocity > current_linear_velocity

        # rospy.loginfo('throttle: %f; brake: %f' % (throttle, brake))

        error_angular_velocity = desired_angular_velocity - current_angular_velocity

        desired_steer = self.yaw_controller.get_steering(
            desired_linear_velocity_modulated, desired_angular_velocity,
            current_linear_velocity)

        current_steer = self.yaw_controller.get_steering(
            desired_linear_velocity_modulated, current_angular_velocity,
            current_linear_velocity)

        steer = self.lowpass_filter.filt((desired_steer - current_steer)) # *self.steer_ratio

        # rospy.loginfo('desired_steer: %f; current_steer: %f; steer: %f' % (desired_steer, current_steer, steer))

        # self.prev_time = time.time()
        return throttle, brake, steer

    # def brake(self, error_in_linear_velocity, current_linear_velocity, vehicle_mass):
    #     # might be more fine tuned, might consider vehicle's mass, and the current velocity, etc.
    #     # might use another PID.
    #     brake_v = -self.brake_coefficient * error_in_linear_velocity  # assume the brake_v should be positive
    #     return max(brake_v, 1.0)
