from pid import PID
import rospy
from yaw_controller import YawController
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704
VERBOSE = False


class Controller(object):
    def __init__(self, *args, **kwargs):

        # TODO: Implement
        self.vehicle_mass = args[0]
        self.fuel_capacity = args[1]
        self.brake_deadband = args[2]
        self.decel_limit = args[3]
        self.accel_limit = args[4]
        self.wheel_radius = args[5]
        self.wheel_base = args[6]
        self.steer_ratio = args[7]
        self.max_lat_accel = args[8]
        self.max_steer_angle = args[9]

        # For maintaining state
        self.previous_time = None
        self.dbw_enabled_currently = False

        # Set up the PID controller
        # TODO: These need to be updated (Twiddle these?)
        self.pid_vel = PID(0.8, 0.0001, 0.1, self.decel_limit, self.accel_limit)

        self.yaw_control = YawController(self.wheel_base, self.steer_ratio, 0.0, self.max_lat_accel, self.max_steer_angle)

        # Set up the low-pass filter for velocity changes
        # TODO: Experiment with these
        self.tau_correction = 0.2
        self.ts_correction = 0.1
        self.low_pass_filter_correction = LowPassFilter(self.tau_correction, self.ts_correction)

        # Calculate the braking torque constant for the vehicle:
        #     wheel radius * (mass of vehicle + mass of fuel)
        self.braking_torque_constant = self.wheel_radius * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY)

    def calculate_elapsed_time(self):
        """
        Calculates the elapsed time since the last time this was called.
        Defaults to 10Hz (1/6 second) on first time called.
        """
        current_time = rospy.get_time()
        elapsed_time = 0.167 # Default: 10Hz = 10/60 = 0.167 seconds
        if (self.previous_time):
            elapsed_time = current_time - self.previous_time
            self.previous_time = current_time
        return elapsed_time

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        target_linear_velocity = args[0]
        target_angular_velocity = args[1]
        current_linear_velocity = args[2]
        is_dbw_enabled = args[3]

        # If DBW isn't currently enabled but we need to enable it
        if is_dbw_enabled:
            if (not self.dbw_enabled_currently):
                # Need to reset state
                self.dbw_enabled_currently = True
                self.previous_time = None
                self.pid_vel.reset()
            else:
                # Continue processing
                pass
        else:
            # Switch it off so we know if we need to reset later
            self.dbw_enabled_currently = False

        # Calculate the velocity cross train error (CTE)
        velocity_cte = target_linear_velocity - current_linear_velocity

        # Get the throttle adjustment for the time step via the PID controller
        elapsed_time = self.calculate_elapsed_time()
        throttle = self.pid_vel.step(velocity_cte, elapsed_time)
        throttle = self.low_pass_filter_correction.filt(throttle)

        brake = 0.  # Default to no brake

        # If the target linear velocity is small, keep the brake on
        if (target_linear_velocity < 0.1):
            throttle = 0.
            brake = abs(self.braking_torque_constant * self.decel_limit)
        elif (throttle < 0.):
            # If throttle < 0, we want to brake instead
            deceleration = abs(throttle)
            throttle = 0.

            # If we're above the brake_deadband value, we can do it
            if (deceleration > self.brake_deadband):
                brake = self.braking_torque_constant * deceleration
            else:
                # Otherwise we can't, so brake remains 0
                pass
        else:
            # Use the calculated throttle value and no brake
            pass

        # Use the YAW controller to calculate a good steering value
        steering = self.yaw_control.get_steering(target_linear_velocity, 
                                                target_angular_velocity, 
                                                current_linear_velocity)

        # In case we need to troubleshoot.
        if (VERBOSE):
            rospy.logerr("throttle = {}".format(throttle))
            rospy.logerr("steering = {}".format(steering))
            rospy.logerr("brake = {}".format(brake))       

        return throttle, brake, steering
