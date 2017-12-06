from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.yaw_controller = YawController(kwargs['wheel_base'],
                                            kwargs['steer_ratio'],
                                            5,  # TODO
                                            kwargs['max_lat_accel'],
                                            kwargs['max_steer_angle'])
        self.vel_pid = PID(kp=5,
                           ki=0.5,
                           kd=0.5,
                           mn=kwargs['decel_limit'],
                           mx=kwargs['accel_limit'])
        self.decel_limit = kwargs['decel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.brake_deadband = kwargs['brake_deadband']
        self.acc_filter = LowPassFilter(tau=3.0, ts=1.0)
        self.steer_filter = LowPassFilter(tau=3.0, ts=1.0)
        self.total_mass = kwargs['vehicle_mass'] + \
            kwargs['fuel_capacity'] * GAS_DENSITY

    def control(self, current_velocity, linear_setpoint, angular_setpoint, delta_t):
        """
        :param current_velocity
        :param linear_setpoint desired linear velocity
        :param angular_setpoints desired angular velocity
        :param delta_t dt
        :return throttle, brake, steer
        """
        cte_velocity = linear_setpoint - current_velocity
        linear_acc = self.vel_pid.step(cte_velocity, delta_t)
        linear_acc = self.acc_filter.filt(linear_acc)
        steer = self.yaw_controller.get_steering(
            linear_setpoint, angular_setpoint, current_velocity)
        steer = self.steer_filter.filt(steer)

        # max brake for slow velocity
        if linear_setpoint < 0.1:
            throttle = 0.0
            brake = abs(self.decel_limit) * self.total_mass * self.wheel_radius
        else:
            # accelerate car
            if linear_acc > 0.0:
                throttle = linear_acc
                brake = 0.0
            # brake car
            else:
                throttle = 0.0
                linear_decel = abs(linear_acc)
                if linear_decel < self.brake_deadband:
                    linear_decel = 0.0
                brake = linear_decel * self.total_mass * self.wheel_radius

        return throttle, brake, steer

    def reset(self):
        self.vel_pid.reset()
