from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, veh_mass, acc_lim, dec_lim, wheel_base, steer_ratio,
                 max_steer, min_speed, max_lat_accel, brake_deadband, wheel_radius):
        self.veh_mass = veh_mass
        self.brake_deadband = brake_deadband
        self.wheel_radius = wheel_radius
        self.dec_lim = dec_lim

        # Within the twist controller, define a yawController instance (class provided by Udacity)
        self.yawCtrl = YawController(wheel_base=wheel_base, steer_ratio=steer_ratio,
                                     min_speed=min_speed, max_lat_accel=max_lat_accel,
                                     max_steer_angle=max_steer)

        # PID Controller
        self.velocity_pid = PID(kp=1.6, ki=0.001, kd=0., mn=dec_lim, mx=acc_lim)

        # Filter
        self.acc_filter = LowPassFilter(4., 1.)

        self.prev_time = rospy.get_time()

    def reset(self):
        self.velocity_pid.reset()

    def get_sample_time(self):
        delta = rospy.get_time() - self.prev_time
        self.prev_time += delta
        return delta

    def control(self, proposed_linear_vel, proposed_angular_vel, current_linear_vel, hold_veh):
        linear_vel_err = proposed_linear_vel - current_linear_vel
        linear_acc = self.velocity_pid.step(linear_vel_err, self.get_sample_time())
        linear_acc = self.acc_filter.filt(linear_acc)
        brake, throttle = 0., 0.

        # HOLD function WILL NOT STOP the vehicle but JUST PREVENT A TAKE-OFF due to some swinging brake/throttle controller or such
        if hold_veh and current_linear_vel < 0.1:
            brake = 10000
        else:
            if linear_acc > 0:
                throttle = linear_acc
            elif abs(linear_acc) > self.brake_deadband:
                brake = abs(linear_acc) * self.veh_mass * self.wheel_radius

        # Get the steering angle from the Udacity-provided YawController
        steer = self.yawCtrl.get_steering(proposed_linear_vel,
                                          proposed_angular_vel,
                                          current_linear_vel)

        # Return throttle, brake, steer
        return throttle, brake, steer
