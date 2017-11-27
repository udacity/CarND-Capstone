from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # Init controller object using supplied arguments
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
        self.min_car_speed = args[10]

        # Init Steering Controller
        self.yaw_control = YawController(self.wheel_base,
                                            self.steer_ratio,
                                            self.min_car_speed,
                                            self.max_lat_accel,
                                            self.max_steer_angle)

        # Init Speed PID Controller
        # Ziegler-Nichols Method
        Tu = 25.
        kP = 2.5
        kI = kP*1.2/Tu
        kD = 3.*kP*Tu/40.

        self.speed_control = PID(kP,kI,kD)
        # set control  limits
        self.speed_control.max = 1.0
        self.speed_control.min = 0.0

        # Init Low Pass Filter
        _hz = 0.0001 # Setting cutoff freq = 0.0001 Hz
        _tau = 1./(_hz*2.*math.pi)
        _s = 10. # higher s-val means more accurate match to original signal, less time smoothing
        self.low_pass = LowPassFilter(_tau,_s)


    def control(self, *args, **kwargs):
        # takes in the current and goal velocities and steering, uses PID's to set throttle, brake, steer
        self.goal_velocity = args[0]
        self.goal_angularv = args[1]
        self.current_velocity = args[2]
        self.current_angularv = args[3] # Not used right now, could be useful for debug
        self.dbw_enabled = args[4]
        self.timestep = 0.02 # If 50 Hz, timestep is 1/50 or 0.02 s - need to confirm

        throttle = 0.0
        brake = 0.0
        steering_angle = 0.0

        if self.dbw_enabled:
            # Steering Controller
            # Get steering angle
            steering_angle = self.yaw_control.get_steering(self.goal_velocity,self.goal_angularv,self.current_velocity)

            # Filter the goal_velocity
            self.current_velocity = self.low_pass.filt(self.current_velocity)

            # Throttle PID
            if (self.goal_velocity > self.min_car_speed)&(self.goal_velocity < 90.):
                # set velocity error
                v_err = (self.goal_velocity - self.current_velocity)/self.goal_velocity
                # get the controller output
                throttle = self.speed_control.step(v_err,self.timestep)
            else: throttle = 0.

        #Resetting PID controller if not DBW
        else:
            self.speed_control.reset()

        # Return throttle, brake, steer
        return throttle, brake, steering_angle
