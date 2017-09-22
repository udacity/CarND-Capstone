from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius,
                 wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        # PID controller used for velocity control
        #self.pid = PID(0.2, 0.0, 3.0, decel_limit, accel_limit)

        # yaw_controller used for steering angle
        self.yaw_controller = YawController(wheel_base, steer_ratio, 5.0, max_lat_accel, max_steer_angle)


    #def control(self, *args, **kwargs):
    def control(self, twist_cmd, velocity_cmd):
        # TODO: Change the arg, kwarg list to suit your needs

        steer = self.yaw_controller.get_steering(twist_cmd.twist.linear.x, twist_cmd.twist.angular.z,
                                                 velocity_cmd.twist.linear.x)

        # Return throttle, brake, steer
        return 0.5, 0., steer
