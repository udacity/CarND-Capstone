import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self, cp):
        self.yaw_controller = YawController(
            wheel_base=cp.wheel_base,
            steer_ratio=cp.steer_ratio,
            min_speed=cp.min_speed,
            max_lat_accel=cp.max_lat_accel,
            max_steer_angle=cp.max_steer_angle)

        self.cp = cp
        self.pid = PID(kp=5, ki=0.5, kd=0.5, mn=cp.decel_limit, mx=cp.accel_limit)
        self.s_lpf = LowPassFilter(tau = 3, ts = 1)
        self.t_lpf = LowPassFilter(tau = 3, ts = 1)

    def reset(self):
        self.pid.reset()

    def control(self, twist_cmd, current_velocity, del_time):
        
        lin_vel = abs(twist_cmd.twist.linear.x)
        ang_vel = twist_cmd.twist.angular.z
        vel_err = lin_vel - current_velocity.twist.linear.x

        next_steer = self.yaw_controller.get_steering(lin_vel, ang_vel, current_velocity.twist.linear.x)
        next_steer = self.s_lpf.filt(next_steer)

        acceleration = self.pid.step(vel_err, del_time)
        acceleration = self.t_lpf.filt(acceleration)

        if acceleration > 0.0:
            throttle = acceleration
            brake = 0.0
        else:
            throttle = 0.0
            deceleration = -acceleration

            if deceleration < self.cp.brake_deadband:
                deceleration = 0.0

            brake = deceleration * (self.cp.vehicle_mass + self.cp.fuel_capacity * GAS_DENSITY) * self.cp.wheel_radius

        # Return throttle, brake, steer
        return throttle, brake, next_steer
