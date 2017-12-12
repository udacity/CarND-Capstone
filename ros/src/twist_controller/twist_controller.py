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

        self.throttle_pid = PID(.15, .00, 0, self.decel_limit, self.accel_limit)
        self.brake_pid = PID(15, 1, 0, 0, 100)
        self.last_velocity_error = 0
        self.last_time = 0
        self.DT = DT
        self.brakeLatch = False

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

            if velocity_error > 2:
                self.brakeLatch = False
            # implement throttle controller
            if self.brakeLatch is False:
                pid_throttle = self.throttle_pid.step(velocity_error, DT, log_handle)
                feedforward_throttle = target_linear_velocity*.01888
                throttle = pid_throttle + feedforward_throttle
                accel_limit = 1 #mps2
                maxThrottle = 0.1962*accel_limit+0.083 # max throttle allowed for a given acceleration limit
                throttle = max(0,min(throttle, maxThrottle))
                brake = 0
                if current_linear_velocity >= .1 and velocity_error < 0 and throttle is 0:
                    self.brakeLatch = True
            #if throttle > maxThrottle:
                #self.throttle_pid.reset()
            else: # we need to brake when throttle PID is saturated at 0 and velocity error is negative
#                decel_target = -velocity_error*.3  #positive 
#                decel_target = min(decel_target, 1)
#                brake = 0.335 * 1080.0/5 * decel_target #braking torque r * m * a
                throttle = 0
                #self.brakeLatch = True
                brake = self.brake_pid.step(-velocity_error, DT, log_handle)        
            if current_linear_velocity < .1 and target_linear_velocity == 0: #clamp at stop
                throttle = 0
                brake = 25
            # implement steering controller
            steering = self.steer_ratio * target_angular_velocity
            self.last_velocity_error = velocity_error
            #args = velocity_error
            self.log_data(log_handle, pid_throttle, feedforward_throttle, velocity_error, DT, decel_target, int(self.brakeLatch))
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
        log_handle.write(','.join(str(arg) for arg in args) + ',')
