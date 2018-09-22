# To access parameters and enable ROS
import rospy
# PID controller
from pid import PID
# yaw controller
from yaw_controller import YawController
# lowpass filter
from lowpass import LowPassFilter

# gas density, really, do we need really this?
GAS_DENSITY = 2.858
# conversion factor MPH into MPS (meter per second)
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, vehicle_mass,
                 fuel_capacity,
                 brake_deadband,
                 decel_limit,
                 accel_limit,
                 wheel_radius,
                 wheel_base,
                 steer_ratio,
                 max_lat_accel,
                 max_steer_angle):
        
        rospy.loginfo('TwistController: Initializing...')
        self.sampling_rate = 50.0
        
        # get parameters for tunning pids
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        # values, decel and accel limit, for regulating discomfortness
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        # for the steering pid
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = 0.1
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        # steering controller
        self.yaw_controller = YawController(wheel_base=self.wheel_base,
                                            steer_ratio=self.steer_ratio,
                                            min_speed=self.min_speed,
                                            max_lat_accel = self.max_lat_accel,
                                            max_steer_angle = self.max_steer_angle)

        # throttle controller
        kp = 0.3
        ki = 0.1
        kd = 0.
        min_throttle = 0.
        max_throttle = 0.2
        self.throttle_controller = PID(kp, ki, kd, min_throttle, max_throttle)

        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = 1 / self.sampling_rate
        self.low_pass_filter_velocity = LowPassFilter(tau, ts)

        self.last_time = rospy.get_time()

        rospy.loginfo('TwistController: Complete to initialize')

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        # Return throttle, brake, steer

        # if dbw is not enabled, you're in the manual mode and you'll
        # have to reset the pid controller, particularly, integral
        # term.
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        # the acceleration value is noisy which you don't want it as
        # it is; need to smooth it out before using it.
        current_velocity = self.low_pass_filter_velocity.filt(current_velocity)

        # get value of steering angle
        steering = self.yaw_controller.get(linear_velocity, angular_velocity, current_velocity)

        error_velocity = linear_velocity - current_velocity
        self.last_velocity = current_velocity

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # get the throttle value
        throttle = self.throttle_controller.step(error_velocity, sample_time)
        brake = 0

        if linear_velocity == 0. and current_velocity < 0.1:
            throttle = 0.
            brake = 700 #N*m
        elif throttle < 0.1 and error_velocity < 0:
            throttle = 0.
            decel = max(error_velocity, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass*self_wheel_radius

        return throttle, brake, steering    
