from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, veh_mass, acc_lim, dec_lim, wheel_base, steer_ratio, max_steer, min_speed, max_lat_accel):
        self.veh_mass = veh_mass
        self.acc_lim = acc_lim
        self.dec_lim = dec_lim
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_steer = max_steer
        self.min_speed = min_speed

        # Within the twist controller, define a yawController instance (class provided by Udacity)
        self.yawCtrl = YawController(wheel_base=wheel_base, steer_ratio=steer_ratio,
                                     min_speed=min_speed, max_lat_accel=max_lat_accel,
                                     max_steer_angle=max_steer)

    def control(self, curr_vel, target_vel, yaw):
        # Done: Change the arg, kwarg list to suit your needs

        # Very simple implementation as a first step
        err = target_vel-curr_vel
        if err > 0:
            throttle = 1. * err
            brake = 0.
        else:
            throttle = 0.
            brake = - 0.1 * err

        # Get the steering angle from the Udacity-provided YawController
        steer = self.yawCtrl.get_steering(target_vel, yaw, curr_vel)

        # Return throttle, brake, steer
        return throttle, brake, steer
