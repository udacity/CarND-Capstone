
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

class Controller(object):
    def __init__(self,  vehicle_mass ,
                        fuel_capacity ,
                        brake_deadband ,
                        decel_limit ,
                        accel_limit ,
                        wheel_radius ,
                        wheel_base ,
                        steer_ratio,
                        max_lat_accel,
                        max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base,steer_ratio,0.1,max_lat_accel,max_steer_angle)
        kp = 0.3
        ki = 0.1 
        kd = 0.
        mn = 0. #Maximum Throttle value
        mx = 0.2 #Maximu Throttle value

        self.throttle_controller = PID(kp,ki,kd,mn,mx)

        tau = 0.5 
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau,ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius


        self.last_time = rospy.get_time()



    def control(self,current_vel,dbw_enabled,linear_vel,angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0 ,0.0 ,0.0 

        current_vel = self.vel_lpf.filt(current_vel)
        # Return throttle, brake, steer
        steering = self.yaw_controller.get_steering(linear_vel,angular_vel,current_vel)

        vel_erro = linear_vel - current_vel
        self.last_vel = current_vel 

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_erro,sample_time)
        brake = 0 

        if linear_vel ==0 and current_vel < 0.1 :
            throttle = 0
            brake = 400 # N*m to hold thr car in place if we are stopped at a light ,acceleration = 1m/sec^2

        elif throttle <.1 and vel_erro <0 :
            throttle = 0
            decel = max(vel_erro,self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque N*m

        return throttle , brake , steering 

