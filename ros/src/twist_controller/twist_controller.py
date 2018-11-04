import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED = 0.15
THROTTLE_MIN = 0
THROTTLE_MAX = 1


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
        min_speed = MIN_SPEED

        self.yaw_controller = YawController(wheel_base,
                                            steer_ratio,
                                            min_speed,
                                            max_lat_accel,
                                            max_steer_angle)

        self.throttle_kp = 0.3
        self.throttle_ki = 0.1
        self.throttle_kd = 0.0
        self.throttle_controller = PID(self.throttle_kp,
                                       self.throttle_ki,
                                       self.throttle_kd,
                                       mn=THROTTLE_MIN,
                                       mx=THROTTLE_MAX)
        
        self.vel_tau = 0.5 # cutoff freq
        self.vel_ts = 0.02 # ms
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
        self.last_vel = current_vel

        now = rospy.get_time()
        dt = now - self.last_time
        self.last_time = now

        throttle = self.throttle_controller.step(vel_error,dt)

        if linear_vel == 0 and current_vel < MIN_SPEED:
            throttle = 0
            brake = 700 # Nm to keep car stationary
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            brake = abs( max(vel_error, self.decel_limit) * self.vehicle_mass * self.wheel_radius)

        return throttle,brake,steer
