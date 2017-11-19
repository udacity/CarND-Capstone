import pid
import lowpass
import yaw_controller
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,
                 max_brake,
                 max_throttle,
                 max_steer_angle,
                 wheel_base,
                 steer_ratio,
                 min_speed,
                 max_lat_accel):
        self.throttle_pid = pid.PID(kp=1.5, ki=0.0, kd=0.01, mn=max_brake, mx=max_throttle)
        self.throttle_filter = lowpass.LowPassFilter(tau=0.0, ts=1.0)

        self.steer_pid = pid.PID(kp=0.5, ki=0.0, kd=0.2, mn=-max_steer_angle, mx=max_steer_angle)
        self.steer_filter = lowpass.LowPassFilter(tau=0.0, ts=1.0)

        self.yaw_controller = yaw_controller.YawController(wheel_base,
                                                           steer_ratio,
                                                           min_speed,
                                                           max_lat_accel,
                                                           max_steer_angle)

    def reset(self):
        rospy.loginfo('Controller has been reset')
        self.throttle_pid.reset()
        self.steer_pid.reset()
        self.throttle_filter.ready = False
        self.steer_filter.ready = False

    def control(self,
                target_linear_vel,
                target_angular_vel,
                current_linear_vel,
                ):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
