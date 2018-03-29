import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
GAS_DENSITY = 2.858
FULL_BRAKE_SPEED = 0.1  # If target velocity is smaller, apply full brake


class Controller(object):
    def __init__(self, vehicle_mass, decel_limit, accel_limit, wheel_radius,
                 wheel_base, steer_ratio, max_lat_accel, max_steer_angle,
                 brake_deadband, fuel_capacity):

        self.velocity_filter = LowPassFilter(3.5, 1.)
        self.velocity_pid = PID(1., 0.001, 0.,
                                mn=decel_limit, mx=0.25)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 1,
                                            max_lat_accel, max_steer_angle)
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband
        self.total_mass = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.decel_limit = decel_limit
        

    def reset(self):
        self.velocity_pid.reset()

    def control(self, twist, velocity, time_diff):
    	self.cmd_vel = twist.twist.linear.x
        self.veh_vel = velocity.twist.linear.x
        self.cmd_ang_vel = twist.twist.angular.z

        velocity_cte = self.cmd_vel - self.veh_vel
        cmd_acc = self.velocity_pid.step(velocity_cte, time_diff)
        steer = self.yaw_controller.get_steering(self.cmd_vel,
                                                 self.cmd_ang_vel,
                                                 self.veh_vel)

        # Apply low-pass filter to the linear acceleration
        cmd_acc = self.velocity_filter.filt(cmd_acc)
        # Keep full brake if target velocity is almost 0
        if self.cmd_vel < FULL_BRAKE_SPEED:
            throttle = 0.0
            brake_torque = self.acceleration_to_torque(abs(self.decel_limit))
        else:
            if cmd_acc > 0.0:
                throttle = cmd_acc
                brake_torque = 0.0
            else:
                throttle = 0.0
                deceleration = -cmd_acc

                # Do not brake if too small deceleration
                if deceleration < self.brake_deadband:
                    deceleration = 0.0

                # Compute brake torque, in Nm
                brake_torque = deceleration * self.total_mass * self.wheel_radius 

        return throttle, brake_torque, steer