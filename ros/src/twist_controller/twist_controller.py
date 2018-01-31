import rospy
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import time

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
STOP_THRESHOLD_VELOCITY = 0.0001
STOP_ACCELERATION       = 0.3


class Controller(object):
    def __init__(self, min_speed, max_speed, accel_limit, decel_limit,
                 max_lat_accel, brake_deadband, wheel_base, wheel_radius,
                 steer_ratio, max_steer_angle, vehicle_mass, fuel_capacity):
        # Initialize member variables.
        self.min_speed       = min_speed
        self.max_speed       = max_speed
        self.accel_limit     = accel_limit
        self.decel_limit     = decel_limit
        self.max_lat_accel   = max_lat_accel
        self.brake_deadband  = brake_deadband
        self.wheel_base      = wheel_base
        self.wheel_radius    = wheel_radius
        self.steer_ratio     = steer_ratio
        self.max_steer_angle = max_steer_angle
        self.vehicle_mass    = vehicle_mass
        self.fuel_capacity   = fuel_capacity
        self.last_control_time = None
        self.last_velocity     = 0.0
        self.dbw_status        = True
        self.brake_mode = False
        # Initialize filter.
        self.velocity_low_pass_filter = LowPassFilter(1.0, 0.02)
        self.vel_ref_low_pass_filter  = LowPassFilter(0.2, 0.02)
        self.acc_ref_low_pass_filter  = LowPassFilter(0.2, 0.02)
        # Initialize controllers.
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed,
                                            max_lat_accel, max_steer_angle)
        self.velocity_controller     = PID(0.7, 0.0, 0.0,
                                           self.decel_limit, self.accel_limit)
        self.acceleration_controller = PID(0.5, 0.4, 0.0,
                                           -1.0, 1.0)

    def control(self, linear_velocity_ref, angular_velocity_ref, current_velocity,
                dbw_status):
        # If the car is controlled by driver, reset controllers once and do
        # not return values.
        if dbw_status != True:
            if self.dbw_status == True:
                self.velocity_controller.reset()
                self.acceleration_controller.reset()
                self.dbw_status = False
            return None, None, None

        # Calculate sample time.
        current_time = time.time()
        if self.last_control_time == None:
            #self.last_control_time = current_time
            #return None, None, None
            sample_time = 0.020
        else:
            sample_time = current_time - self.last_control_time
        self.last_control_time = current_time

        # Initialize commands.
        throttle = 0.0
        brake    = 0.0

        # Filter current velocity.
        linear_velocity_ref = self.vel_ref_low_pass_filter.filt(linear_velocity_ref)
        current_velocity = self.velocity_low_pass_filter.filt(current_velocity)
        current_acceleration = (current_velocity - self.last_velocity) / sample_time

        # Control steering.
        steering = self.yaw_controller.get_steering(linear_velocity_ref, angular_velocity_ref,
                                                    current_velocity)

        # Control velocity with output acceleration.
        velocity_error = linear_velocity_ref - current_velocity
        acceleration_ref = self.velocity_controller.step(velocity_error, sample_time)
        acceleration_ref = self.acc_ref_low_pass_filter.filt(acceleration_ref)
        # Control acceleration with output throttle.
        acceleration_error = acceleration_ref - current_acceleration
        throttle = self.acceleration_controller.step(acceleration_error, sample_time)

        # Switch between acceleration and braking.
        if (acceleration_ref < -self.brake_deadband) or ((acceleration_ref < 0) and (self.brake_mode == True)):
            # Braking.
            if self.brake_mode == False:
                self.acceleration_controller.reset()
            brake = ((self.vehicle_mass + self.fuel_capacity * GAS_DENSITY)
                     * (-5) * self.wheel_radius)
            if throttle > 0:
                throttle = 0
            brake = throttle * brake
            throttle = 0
            self.brake_mode = True
        else:
            if self.brake_mode == True:
                self.acceleration_controller.reset()
            if throttle < 0:
                throttle = 0
            # Accelerating.
            self.brake_mode = False

        # Debug.
        #p, i, dt = self.acceleration_controller.get_values()
        #rospy.logwarn('twist_controller.py - control - p: %5.3f, i: %5.3f, sum: %5.3f, dt: %5.3f',
        #               p, i, p+i, dt)
        #rospy.logwarn('twist_controller.py - control - ref_vel: %5.3f, cur_vel: %5.3f',
        #              linear_velocity_ref, current_velocity)
        #rospy.logwarn('twist_controller.py - control - cur_acc: %5.3f, ref_acc: %5.3f, throttle: %5.3f, brake: %5.3f, cur_vel: %5.3f, ref_vel: %5.3f, dt: %5.3f, p: %5.3f, i: %5.3f, sum: %5.3f, mode: %i',
        #              current_acceleration, acceleration_ref, throttle, brake, current_velocity, linear_velocity_ref, sample_time * 1000.0, p, i, p+i, self.brake_mode)

        self.last_velocity = current_velocity

        # Holding.
        if (linear_velocity_ref < STOP_THRESHOLD_VELOCITY) and (current_velocity < self.min_speed):
            brake = ((self.vehicle_mass + self.fuel_capacity * GAS_DENSITY)
                     * STOP_ACCELERATION * self.wheel_radius)
            throttle = 0

        return throttle, brake, steering
