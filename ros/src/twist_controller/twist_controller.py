import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import constants as const

class Controller(object):
    def __init__(self,
                 vehicle_mass,
                 brake_deadband,
                 decel_limit,
                 accel_limit,
                 wheel_radius,
                 wheel_base,
                 steer_ratio,
                 max_lat_accel,
                 max_steer_angle):


        self.decel_limit = decel_limit
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        min_speed = const.CLOSE_TO_ZERO_SPEED

        self.yaw_controller = YawController(wheel_base,
                                            steer_ratio,
                                            min_speed,
                                            max_lat_accel,
                                            max_steer_angle)

        self.throttle_controller = PID(const.THROTTLE_KP,
                                       const.THROTTLE_KI,
                                       const.THROTTLE_KD,
                                       mn=const.THROTTLE_MIN,
                                       mx=const.THROTTLE_MAX)
        
        self.vel_tau = const.LPF_CUTTOFF_FREQ
        self.vel_ts = const.LPF_TS
        self.vel_lowPassFilter = LowPassFilter(self.vel_tau, self.vel_ts)

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_en, linear_vel, angular_vel):
        throttle = 0
        brake = 0
        steer = 0

        if not dbw_en:
            self.throttle_controller.reset()
            return 0., 0., 0.
        current_vel = self.vel_lowPassFilter.filt(current_vel)
        steer = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel

        now = rospy.get_time()
        dt = now - self.last_time
        self.last_time = now

        throttle = self.throttle_controller.step(vel_error,dt)


        if abs(linear_vel) < const.CLOSE_TO_ZERO_SPEED and current_vel < const.CLOSE_TO_ZERO_SPEED:
            rospy.loginfo("linear_vel = %s",linear_vel)
            throttle = 0
            self.throttle_controller.reset()
            brake = const.BRAKE_STATIONARY_FORCE 
        elif throttle < const.CLOSE_TO_ZERO_SPEED and vel_error < const.NEAR_ZERO_FLOAT:
            throttle = 0
            brake = abs( max(vel_error, self.decel_limit) * self.vehicle_mass * self.wheel_radius)

        return throttle,brake,steer
