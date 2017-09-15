from yaw_controller import YawController
from lowpass import LowPassFilter
from math import atan
from pid import PID


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,kp_vel, kp_throttle, ki_throttle, min_speed, 
            vehicle_mass, fuel_capacity, brake_deadband, 
            decel_limit, accel_limit,
            wheel_radius, wheel_base, 
            steer_ratio, max_lat_accel, max_steer_angle, rate ):
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
        self.cur_acc = 0
        self.rate = rate
        self.accel_filter = LowPassFilter(0.2, 1./self.rate)
        self.yaw_controller = YawController(wheel_base, 
        									steer_ratio,
        									min_speed,
        									max_lat_accel,
        									max_steer_angle)
        self.steer_pid = PID(0.5, 0.001, 0.0)
        self.throttle_pid = PID(kp_throttle, ki_throttle, 0.0)


        pass

    def control(self, cmd_lin_vel, cmd_ang_vel, cur_lin_vel, cur_ang_vel, delta_t, dbw_enabled):
    	brake = 0.
    	throttle = 0.

        # steering = self.yaw_controller.get_steering(cmd_lin_vel, cmd_ang_vel, cur_lin_vel)
        print("cmd_ang_vel: ", cmd_ang_vel)
        print("cmd_lin_vel: ", cmd_lin_vel)

        steer_error = cmd_ang_vel - cur_ang_vel

        # steering = self.steer_pid.step(steer_error, delta_t)
        steering = 10*atan(self.wheel_base * cmd_ang_vel / (cmd_lin_vel+0.01) ) * self.steer_ratio
        # print("steering: ", steering)
        cmd_acc = self.kp_vel * (cmd_lin_vel - cur_lin_vel)

        if cmd_acc > self.accel_limit:
        	cmd_acc = self.accel_limit
        elif cmd_acc < self.decel_limit:
        	cmd_acc = self.decel_limit

        # self.cur_acc = ( (cur_lin_vel - self.last_cur_lin_vel)*self.rate ) * 0.1 + self.cur_acc * 0.9
        # self.cur_acc = self.accel_filter.filt(self.cur_acc)
        print("cmd_acc: ", cmd_acc)
        print("cur_acc: ", self.cur_acc)

        # self.last_cur_lin_vel = cur_lin_vel
        # self.last_acc = self.cur_acc 

        if(dbw_enabled==False):
            self.steer_pid.reset()
            self.throttle_pid.reset()

        throttle_error = cmd_acc

        brake = -throttle_error*self.vehicle_mass*self.wheel_radius
        # throttle = p_error*self.kp_throttle + self.i_error
        throttle = self.throttle_pid.step(throttle_error, delta_t)

        if(cmd_acc < 0):
        	if(brake < self.brake_deadband):
        		brake = 0.
        	throttle = 0.
        else:
        	brake = 0.


        print("throttle: ", throttle)

        return throttle, brake, steering
