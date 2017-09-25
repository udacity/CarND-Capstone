from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import rospy
import time

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius,
                 wheel_base, steer_ratio, max_lat_accel, max_steer_angle, kp, ki, kd):
        rospy.logwarn("wheel_base: {}\tsteer_ratio: {}\tmax_lat_accel: {}\tmax_steer_angle: {}\n".format(wheel_base, steer_ratio, max_lat_accel, max_steer_angle))

        # PID controller used for velocity control
        # using kp, ki, kd from params file
        self.pid = PID(kp, ki, kd, decel_limit, accel_limit)

        self.last_t = None

        # yaw_controller used for steering angle
        # self.low_pass_filter = LowPassFilter(0.96, 1.0)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 5.0, max_lat_accel, max_steer_angle)

    def control(self, twist_cmd, velocity_cmd):
        # TODO: Change the arg, kwarg list to suit your needs

        if self.last_t is None:
            self.last_t = time.time()
            return 0.0, 0.0, 0.0

        # Calculate steering
        twist_linear_x = twist_cmd.twist.linear.x
        #twist_angular_z = self.low_pass_filter.filt(twist_cmd.twist.angular.z)
        twist_angular_z = twist_cmd.twist.angular.z
        current_velocity_x = velocity_cmd.twist.linear.x

        delta_t = self.last_t - time.time()
        self.last_t = time.time()
        # calculating error
        error_v = twist_linear_x - current_velocity_x
        # rospy.logwarn("twist_linear_x={}\tcurrent_v={}\terror_v={}\n".format(twist_linear_x, current_velocity_x, error_v))
        # getting throttle from pid controller with error_v and delta_t
        # keeping it between 0.0 and 1.0
        throttle = max(0.0, min(1.0, self.pid.step(error_v, delta_t)))

        if error_v < 0:
            # error_v smaller zero means braking
            # limit to 1.0 and set throttle to 0.0
            brake = max(error_v, 1.0)
            throttle = 0.0
        else:
            brake = 0.0

        steer = self.yaw_controller.get_steering(twist_linear_x, twist_angular_z, current_velocity_x)

        # throttle = 0.25
        # brake = 0.0
        return throttle, brake, (steer * 10.0)
