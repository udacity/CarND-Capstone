from yaw_controller import YawController
from lowpass import LowPassFilter
from math import atan


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,kp_vel, kp_throttle, ki_throttle, min_speed, 
            vehicle_mass, fuel_capacity, brake_deadband, 
            decel_limit, accel_limit,
            wheel_radius, wheel_base, 
            steer_ratio, max_lat_accel, max_steer_angle ):
        self.i_error = 0
        self.kp_vel = kp_vel
        self.kp_throttle = kp_throttle
        self.ki_throttle = ki_throttle
        self.last_cur_lin_vel = 0
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband
        self.steer_ratio = steer_ratio
        self.wheel_base = wheel_base
        self.accel_filter = LowPassFilter(0.2, 0.02)
        self.yaw_controller = YawController(wheel_base, 
        									steer_ratio,
        									min_speed,
        									max_lat_accel,
        									max_steer_angle)


        pass

    def control(self, cmd_lin_vel, cmd_ang_vel, cur_lin_vel, dbw_enabled):
    	brake = 0.
    	throttle = 0.

        # steering = self.yaw_controller.get_steering(cmd_lin_vel, cmd_ang_vel, cur_lin_vel)
        steering = atan(self.wheel_base * cmd_ang_vel / (cmd_lin_vel+0.01) ) * self.steer_ratio
        cmd_acc = self.kp_vel * (cmd_lin_vel - cur_lin_vel)

        if cmd_acc > self.accel_limit:
        	cmd_acc = self.accel_limit
        elif cmd_acc < self.decel_limit:
        	cmd_acc = self.decel_limit

        self.cur_acc = cur_lin_vel - self.last_cur_lin_vel
        self.cur_acc = self.accel_filter.filt(self.cur_acc)
        # print("cmd_acc: ", cmd_acc)
        # print("cur_acc: ", self.cur_acc)

        self.last_cur_lin_vel = cur_lin_vel 

        if(dbw_enabled==False):
        	self.i_error = 0

        p_error = cmd_acc - self.cur_acc
        self.i_error += p_error

        # calculate brake
        if(cmd_acc < 0):
        	brake = -p_error*self.vehicle_mass*self.wheel_radius
        	if(brake < self.brake_deadband):
        		brake = 0.
        	throttle = 0.
        else:
        	brake = 0.
        	# calculate throttle
	        throttle = p_error*self.kp_throttle + self.i_error*self.ki_throttle

        return throttle, brake, steering
