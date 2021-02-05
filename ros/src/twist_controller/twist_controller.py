#Bibliotheque necessaire 

# Source: DBW Walkthrough
# Lesson 8 of Project section in Course

from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):      
        
        # TODO: Implement
        #pass
        #create yaw controller object
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        #Parameters of throttle PID 
        kp = 0.3
        ki = 0.1
        kd = 0.
        min = 0.  #Minimun throttle value
        max = 0.3 #Maximun throttle value
        
        #Throttle PID object
        self.throttle_controller = PID(kp, ki, kd, min, max)
        
        #Parameters of velocity low pass 
        tau = 0.5  
        ts = 0.02  
        
        #Velocity low pass object
        self.vel_lpf = LowPassFilter(tau, ts)
        
        #Parameters of car
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        self.last_time = rospy.get_time()

    #In the development of this control function which sends us the acceleration, speed and direction commands we use a low pass filter to suppress all the high frequencies in the speed messages and a yaw controller for the acceleration...  
    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        
        #Here we first check if the vehicle's cable transmission is activated or not
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        #Received speed filtering
        current_vel = self.vel_lpf.filt(current_vel)
        
        #Call of the yaw controller function
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        #Application of PID controller
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0
        
        #This condition loop allows us to tell the car what to do in certain conditions
        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0.0
            brake = 700  # N*m - to hold the car in place 
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0.0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius   # Torque N*m 

        return throttle, brake, steering
        #return 1., 0., 0.
