from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.yaw_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

    def control(self, target_linear_velocity, target_angular_velocity, current_linear_velocity):
        # TODO - implement proper throttle and brake measurements

        # Return throttle, brake, steer
        return 0., 0., self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)
