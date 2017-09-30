<<<<<<< HEAD
# Credits: Wonderful SDC Slack Community

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
||||||| merged common ancestors
=======
from pid import PID
from yaw_controller import YawController
>>>>>>> upstream/master

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

<<<<<<< HEAD
||||||| merged common ancestors

=======
import math

>>>>>>> upstream/master
class Controller(object):
<<<<<<< HEAD
    def __init__(self, *args, **kwargs):
        	
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

	self.min_speed = 0

	self.speed_controller = PID(0.5, 0.02, 0.2, 							self.decel_limit,self.accel_limit)
	self.steering_controller = PID(5, 0.05, 1, -0.5, 0.5)

	self.tau = 0.2
	self.ts = 0.1
	self.lpf = LowPassFilter(self.tau, self.ts)
	
	self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)

    def control(self, *args, **kwargs):
 
	target_linear_vel = args[0]
	target_angular_vel = args[1]
	current_linear_vel = args[2]
	current_angular_vel = args[3]
	dbw_en = args[4]
	dt = args[5]

	linear_vel_cte = (target_linear_vel - current_linear_vel)
	angular_vel_cte = target_angular_vel

	if dbw_en is False: #PID must not accumulate error under manual control
	    self.speed_controller.reset()
	    self.steering_controller.reset()

	linear_vel = self.speed_controller.step(linear_vel_cte, dt)
	linear_vel = self.lpf.filt(linear_vel)
	
	throttle = 0
	brake = 0
	
	if target_linear_vel < 0.01:
	    brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * 					self.wheel_radius * abs(self.decel_limit)
	    throttle = 0

	elif linear_vel > 0:
	    throttle = linear_vel
	    brake = 0
	else:
	    
	    brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * 					self.wheel_radius * abs(linear_vel)
	    throttle = 0
  	
	corrective_steer = self.steering_controller.step(angular_vel_cte, dt)
	predictive_steer = self.yaw_controller.get_steering(target_linear_vel, 					target_angular_vel, current_linear_vel)
	steer = corrective_steer + predictive_steer

        return throttle, brake, steer
||||||| merged common ancestors
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
=======

    def __init__(self, vehicle_mass, wheel_radius, accel_limit, decel_limit, 
                       wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # self.speed_controller = PID(5, 0.05, 1, -0.5, 0.5)

        # use a separate speed controller, than from PID
        self.speed_controller = SpeedController(
                                vehicle_mass,
                                wheel_radius,
                                accel_limit,
                                decel_limit)
        # self.speed_controller = PID(0.5, 0.02, 0.2)
        # self.steering_controller = PID(5, 0.05, 1, -0.5, 0.5)
        self.steering_controller = PID(0.5, 0.05, 0.1, -0.35, 0.35)
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

    def control(self, target_velocity, current_velocity, dbw_enabled, dt):
        target_linear_velocity, target_angular_velocity = target_velocity
        current_linear_velocity, current_angular_velocity = current_velocity

        # `rostopic echo /twist_cmd` says target linear velocity is fixed to 11.
        # but target angular velocity is changing based on vehicle's orienta
        linear_velocity_cte = target_linear_velocity - current_linear_velocity
        angular_velocity_cte = target_angular_velocity

        if not dbw_enabled: # manual driving
            self.speed_controller.reset()
            self.steering_controller.reset()

        corrective_steer = self.steering_controller.step(angular_velocity_cte, dt)
        predictive_steer = self.yaw_controller.get_steering(target_linear_velocity,
                                                            target_angular_velocity,
                                                            current_linear_velocity)
        steer = corrective_steer + predictive_steer

        throttle, brake = self.speed_controller.step(linear_velocity_cte, dt)

        # linear_velocity = self.speed_controller.step(linear_velocity_cte, dt)
        # throttle = 0
        # brake = 0

        # if linear_velocity > 0:
        #     throttle = linear_velocity
        # else:
        #     # brake = abs(linear_velocity) * 200000.
        #     acceleration = max(-5, linear_velocity_cte / dt)
        #     torque = 1736.35 * acceleration * 0.2413
        #     brake = min(abs(torque), 20000.)
        return throttle, brake, steer






class SpeedController(object):
    """ A Speed Controller class 
    Code modified from 
    https://github.com/kung-fu-panda-automotive/carla-driver/blob/master/ros/src/twist_controller/speed_controller.py
    """

    MAX_THROTTLE_TORQUE = 10.0
    MAX_BREAK_TORQUE = 20000.0

    def __init__(self, vehicle_mass, wheel_radius, accel_limit, decel_limit):
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit

    def step(self, error, sample_time):
        """
        """
        # calculate the acceleration based on the time
        # we need to make the change happen
        sample_time = 0.5
        acceleration = error / sample_time
        # apply limits to acceleration
        if acceleration > 0:
            acceleration = min(self.accel_limit, acceleration)
        else:
            acceleration = max(self.decel_limit, acceleration)
        # calculate torque = M*acc*R
        torque = self.vehicle_mass * acceleration * self.wheel_radius
        throttle, brake = 0, 0
        if torque > 0:
            # As documented, throttle is the percent of max torque applied
            throttle, brake = max(0.1, min(1.0, torque / SpeedController.MAX_THROTTLE_TORQUE)), 0.0
        else:
            # brake is the torque we need to apply
            throttle, brake = 0.0, min(abs(torque), SpeedController.MAX_BREAK_TORQUE)

        return throttle, brake

    def reset(self):
        pass
>>>>>>> upstream/master
