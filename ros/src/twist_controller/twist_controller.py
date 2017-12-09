import rospy
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
DT = 1./50.


class Controller(object):
    def __init__(self, steer_ratio, decel_limit, accel_limit):

        self.steer_ratio = steer_ratio
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit

        self.throttle_pid = PID(1, 0.1, 0.1, self.decel_limit, self.accel_limit)
        self.brake_pid = PID(1, 0.1, 0.1, self.decel_limit, self.accel_limit)
        self.last_velocity_error = 0
        self.last_time = 0
        self.DT = DT

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

            if self.is_change_acc(velocity_error):
                self.throttle_pid.reset()
                self.brake_pid.reset()

            # implement throttle controller
            if velocity_error >= 0:
                throttle = self.throttle_pid.step(velocity_error, DT)
                brake = 0

            # implement brake controller
            else:
                throttle = 0
                brake = self.brake_pid.step(-velocity_error, DT)

            # implement steering controller
            steering = self.steer_ratio * target_angular_velocity

            self.last_velocity_error = velocity_error
            #args = velocity_error
            self.log_data(log_handle, velocity_error, DT)

        else:
            throttle = 0
            brake = 0
            steering = 0
            rospy.loginfo("dbw_status false")

        return throttle, brake, steering

    def is_change_acc(self, velocity_error):

        is_switch_brake = (self.last_velocity_error >= 0) and (velocity_error < 0)
        is_switch_acc = (self.last_velocity_error <= 0) and (velocity_error > 0)

        return is_switch_brake or is_switch_acc
    
    def log_data(self, log_handle, *args):
        #log_handle.write(','.join(str(arg) for arg in args)+',' )
        log_handle.write(','.join(str(arg) for arg in args) + ',')
