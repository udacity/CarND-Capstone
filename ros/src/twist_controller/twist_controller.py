
import rospy
from pid import PID
from lowpass import LowPassFilter

from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704



class Controller(object):
    def __init__(self, EgoParam):
        # TODO: Implement

        self.yaw_controller = YawController(
        wheel_base= EgoParam.wheel_base,
        steer_ratio = EgoParam.steer_ratio,
        min_speed=0.1,
        max_lat_accel= EgoParam.max_lat_accel,
        max_steer_angle= EgoParam.max_steer_angle)
        
        self.EgoParam=EgoParam


        self.throttle_controller = PID(kp=0.3,ki=0.1,kd=0.0,mn = EgoParam.decel_limit,mx=EgoParam.accel_limit)

        tau=0.5 # 1/(2pi*tau) = cutoff frequency
        ts = 0.02 # Sample Time

        self.vel_lpf = LowPassFilter(tau,ts)
        self.last_time = rospy.get_time()
        pass

    def control(self,current_vel,dbw_enabled,linear_vel,angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0,0.0,0.0
        current_vel= self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel,angular_vel,current_vel)

        vel_error = linear_vel-current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time= current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake=0

        if linear_vel ==0.0 and current_vel<0.1:
            throttle=0
            brake=400 #Nm - to hold the car in place if we are stopped at a light. acceleration-1m/s^2
        elif throttle<0.1 and vel_error<0:
            throttle=0
            decel=max(vel_error, self.EgoParam.decel_limit)
            brake=abs(decel)*self.EgoParam.vehicle_mass *self.EgoParam.wheel_radius

        return throttle, brake, steering
