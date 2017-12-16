import rospy
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
DT = 1./50.


class Controller(object):
    def __init__(self, steer_ratio, decel_limit, accel_limit, max_steer_angle, wheel_base, max_let_accel):

        self.steer_ratio = steer_ratio
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.max_steer_angle = max_steer_angle
        self.max_let_accel = max_let_accel
        self.wheel_base = wheel_base

        self.throttle_pid = PID(.15, .00, 0, self.decel_limit, self.accel_limit)
        self.brake_pid = PID(15, 1, 0, 0, 100)
        self.steer_pid = PID(2.0, 0.0005, 2.0, -self.max_steer_angle, self.max_steer_angle)
        self.last_velocity_error = 0
        self.last_time = 0
        self.DT = DT
        self.brakeLatch = False

        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio,
                                            0, self.max_let_accel, self.max_steer_angle)

    def control(self, target_linear_velocity, target_angular_velocity,
                current_linear_velocity, dbw_status, log_handle):
        '''Defines target throttle, brake and steering values'''

        if dbw_status:
            # Update DT
            new_time = rospy.get_rostime()
            if self.last_time:  # The first time, we are not able to calculate DT
                self.DT = (new_time - self.last_time).to_sec()
            self.last_time = new_time

            velocity_error = target_linear_velocity - current_linear_velocity
            pid_throttle, feedforward_throttle, decel_target = 0, 0, 0

            # implement longitudinal controller

            # if we're going too slow, release the brakes (if they are applied)
            # This essentially adds hysterisis: the brakes are only enabled if our velocity error is negative, and only
            # released if the velocity error is positive 2 or greater.
            if velocity_error > 2:
                self.brakeLatch = False
            if self.brakeLatch is False:
                pid_throttle = self.throttle_pid.step(velocity_error, self.DT)
                feedforward_throttle = target_linear_velocity*.01888 #based on desired speed, predict how much throttle we need at steady state
                throttle = pid_throttle + feedforward_throttle
                accel_limit = 1 #mps2
                maxThrottle = 0.1962*accel_limit+0.083 # max throttle allowed for a given acceleration limit
                throttle = max(0,min(throttle, maxThrottle))  # saturate throttle if it exceeds acceleration limits
                brake = 0
                if current_linear_velocity >= .1 and velocity_error < 0 and throttle is 0:
                    self.brakeLatch = True
            # we need to brake when throttle PID is saturated at 0 and velocity error is negative. In other words, when
            # we can't slow down any more (throttle is already zero), and we want to slow down more (speed error is
            # negative), then we need to use the brakes
            else:
                throttle = 0
                brake = self.brake_pid.step(-velocity_error, self.DT)
            # If we're about to come to a stop, clamp the brake command to some value to hold the vehicle in place
            if current_linear_velocity < .1 and target_linear_velocity == 0:
                throttle = 0
                brake = 25

            # implement steering controller
            steer_error = self.yaw_controller.get_steering(target_linear_velocity,
                                                     target_angular_velocity,
                                                     current_linear_velocity)

            steering = self.steer_pid.step(steer_error, self.DT)

            self.last_velocity_error = velocity_error

        else:
            self.brakeLatch = False
            self.throttle_pid.reset()
            self.brake_pid.reset()
            self.steer_pid.reset()
            throttle, brake, steering = 0, 0, 0
            pid_throttle, feedforward_throttle, velocity_error, decel_target = 0, 0, 0, 0

        # Log data
        throttle_P, throttle_I, throttle_D = self.throttle_pid.get_PID()
        brake_P, brake_I, brake_D = self.brake_pid.get_PID()
        steer_P, steer_I, steer_D = self.steer_pid.get_PID()
        self.log_data(log_handle, throttle_P, throttle_I, throttle_D, brake_P, brake_I, brake_D, steer_P, steer_I, steer_D,
                          pid_throttle, feedforward_throttle, velocity_error, self.DT, decel_target, int(self.brakeLatch))

        return throttle, brake, steering
    
    def log_data(self, log_handle, *args):
        log_handle.write(','.join(str(arg) for arg in args) + ',')
