from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,
                 wheel_base,
                 steer_ratio,
                 min_speed,
                 max_lat_accel,
                 max_steer_angle,
                 accel_limit,
                 decel_limit,
                 loop_frequency,
                 vehicle_mass,
                 wheel_radius):
        # TODO: Implement
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_steer_angle = max_steer_angle

        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius

        self.steering_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.throttle_controller = PID(0.15, 0.0, 0.05, mn=decel_limit, mx=accel_limit)

        self.low_pass_filter = LowPassFilter(2.0 / 1000.0, 1.0 / loop_frequency)

        self.last_timestamp = None



    def control(self, target_angular_velocity, target_linear_velocity, current_angular_velocity, current_linear_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.reset()
            return 0., 0., 0.

        steer = self.steering_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)
        # steer = 0.
        throttle = 0.
        brake = 0.

        current_timestamp = rospy.Time.now()
        if self.last_timestamp != None:
            dt =  (current_timestamp - self.last_timestamp).nsecs / 1e9
            cte = target_linear_velocity - current_linear_velocity
            acceleration = self.throttle_controller.step(cte, dt)

            # filtvalue = self.low_pass_filter.filt(acceleration)
            # if self.low_pass_filter.ready:
            #     acceleration = self.low_pass_filter.get()

            if acceleration > 0:
                throttle = acceleration
            else:
                brake = self.vehicle_mass * abs(acceleration) * self.wheel_radius


        self.last_timestamp = current_timestamp
        rospy.loginfo('SENDING - [throttle,brake,steer]:[{:.4f},{:.4f},{:.4f}], [cA,cL]:[{:.4f},{:.4f}]m [tA, tL]:[{:.4f},{:.4f}]'.format(throttle, brake, steer,current_angular_velocity, current_linear_velocity,target_angular_velocity, target_linear_velocity))
        return throttle, brake, steer

    def reset(self):
        """
        Reset the controller's state.
        """
        self.throttle_controller.reset()
        self.low_pass_filter.reset()
