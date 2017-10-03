from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,decel_limit, accel_limit, wheel_radius,
                 wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        
        #PID setup
        self.kp = 0.8
        self.ki = 0.0001
        self.kd = 0.5
        
        self.long_PID = PID(self.kp,self.ki,self.kd,decel_limit,accel_limit)
        
        min_speed = 0.1
        self.yaw_ctrl = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        
        #Values for Braking
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband
        self.tot_veh_mass = vehicle_mass + fuel_capacity*GAS_DENSITY

    def control(self, target_long_vel,target_yaw_rate,cur_long_vel,dbw_enabled,dt):
        
        throttle = 0.0
        brake_tq = 0.0
        steer_angle = 0.0
        
        #Longitudinal Control
        long_vel_err = target_long_vel -cur_long_vel
        
        target_long_accel = self.long_PID.step(long_vel_err,dt)
        
        if target_long_accel > 0.0:
            throttle = target_long_accel
            
        else:
            throttle = 0.0
            
            if abs(target_long_accel) > self.brake_deadband:
                brake_tq = abs(target_long_accel) *self.tot_veh_mass*self.wheel_radius
        
        #Lateral Control
        steer_angle = self.yaw_ctrl.get_steering(target_long_vel,target_yaw_rate,cur_long_vel)
        
        return throttle,brake_tq,steer_angle
