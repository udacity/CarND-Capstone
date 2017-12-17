import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

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

        self.throttle_pid = PID(0.1, 0.001, 0, 0, 0.4)
        self.brake_pid = PID(60., 1, 0, 0, 100)
        self.steer_filter = LowPassFilter(0.2, DT)
        self.last_time = 0
        self.DT = DT
        self.brakeLatch = False

        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio,
                                            0, self.max_let_accel, self.max_steer_angle)

    def control(self, target_linear_velocity, target_angular_velocity,
                current_linear_velocity, dbw_status, log_handle):
        '''Defines target throttle, brake and steering values'''

        # Update DT
        new_time = rospy.get_rostime()
        if self.last_time:  # The first time, we are not able to calculate DT
            self.DT = (new_time - self.last_time).to_sec()
        self.last_time = new_time
            
        if dbw_status:            
            velocity_error = target_linear_velocity - current_linear_velocity

            # if we're going too slow, release the brakes (if they are applied)
            # This essentially adds hysterisis: the brakes are only enabled if our velocity error is negative, and only
            # released if the velocity error is positive 2 or greater.
            if velocity_error > 1:
                self.brakeLatch = False
            if not self.brakeLatch:
                throttle = self.throttle_pid.step(velocity_error, self.DT)
                brake = 0
                self.brake_pid.reset()
                # if we go too fast and we cannot decrease throttle, we need to start braking
                if (velocity_error < 0 and throttle is 0) or (velocity_error < -1):
                    self.brakeLatch = True
            else:
                # We are currently braking
                throttle = 0
                self.throttle_pid.reset()
                brake = self.brake_pid.step(-velocity_error, self.DT)
            # If we're about to come to a stop, clamp the brake command to some value to hold the vehicle in place
            if current_linear_velocity < .1 and target_linear_velocity == 0:
                throttle = 0
                brake = 0.5

            # implement steering controller
            steer_target = self.yaw_controller.get_steering(target_linear_velocity,
                                                     target_angular_velocity,
                                                     current_linear_velocity)
            steering = self.steer_filter.filt(steer_target)

        else:
            self.brakeLatch = False
            self.throttle_pid.reset()
            self.brake_pid.reset()
            self.steer_filter.reset()
            throttle, brake, steering = 0, 0, 0
            pid_throttle, velocity_error = 0, 0

        # Log data
        throttle_P, throttle_I, throttle_D = self.throttle_pid.get_PID()
        brake_P, brake_I, brake_D = self.brake_pid.get_PID()
        steer_P, steer_I, steer_D = 0, 0, 0
        self.log_data(log_handle, throttle_P, throttle_I, throttle_D, brake_P, brake_I, brake_D,
                          velocity_error, self.DT, int(self.brakeLatch))

        return throttle, brake, steering
    
    def log_data(self, log_handle, *args):
        log_handle.write(','.join(str(arg) for arg in args) + ',')
