from yaw_controller import YawController
from pid import PID
import rospy

from lowpass import LowPassFilter #TODO: find out how to use it

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        '''args = [0:wheel_base,    1:steer_ratio,
                   2:max_lat_accel, 3:max_steer_angle,
                   4:decel_limit,   5:accel_limit
                   6:brake_deadband,7:vehicle_mass ,
                   8:fuel_capacity, 9:wheel_radius ,
                   10:sample_rate
        ]'''
        
        self.brake_deadband = args[6]
        self.vehicle_mass   = args[7]
        self.fuel_capacity  = args[8]
        self.wheel_radius = args[9]

        self.min_speed = (rospy.get_param('/waypoint_loader/velocity') * 1000.) / (60. * 60.) #kmph to mps
        self.yaw_controller = YawController(args[0],#wheel_base
                                            args[1],#steer_ratio
                                            0,
                                            args[2],#max_lat_accel
                                            args[3])#max_steer_angle
        self.sample_time = 1.0/args[6] # 1/sample_rate
        self.lowpass_tau = .03#1 is default value, it should be the max steering value allowed to avoid jerk
        self.lowpass_throttle = LowPassFilter(self.lowpass_tau, self.sample_time)
        self.pid_throttle = PID( 3.0, 0.0, 0.5, args[4], args[5] )
        self.pid_steer = PID( 2.0, 0.15, 0.6, (-1*args[3]), args[3] ) 
        
        pass
    
    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        ''' args = [0:proposed_linear_velocity,
                    1:proposed_angular_velocity,
                    2:current_linear_velocity,  
                    3:current_angular_velocity
                    4:is_dbw_enabled] from DBWNode dbw_node.py
        '''
        if args[4] : #if is_dbw_enabled
            #rospy.logdebug("proposed_angular_velocity:%f",args[1])    
            #rospy.logdebug("current_angular_velocity:%f",args[3])
            #steer =  self.yaw_controller.get_steering(args[0],args[1],args[2])#bad and delayed but smooth
            steer_CTE = args[1]-args[3]
            #rospy.logdebug("steer_CTE :%f",steer_CTE)
            
            steer = self.pid_steer.step(steer_CTE, self.sample_time)
            #rospy.logdebug("steer :%f",steer)
            #steer = self.lowpass_steer.filt(steer)#TODO: test before uncomment and commit
            throttle_CTE = args[0]-args[2] #proposed_linear_velocity - current_linear_velocity
            throttle = self.pid_throttle.step(throttle_CTE,self.sample_time)#1/15 or 1/50
            brake = 0
            #Brakes system
            if throttle < 0:
                #code refereace : https://discussions.udacity.com/t/what-is-the-range-for-the-brake-in-the-dbw-node/412339
                brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * (throttle*-1) * self.wheel_radius
                throttle = 0
                #rospy.logdebug("^^^^BREAK^^^^")    
            else : 
                brake = 0
                #rospy.logdebug("throttle = %f",throttle)
            #car used to NEVER stop on trafic lights, it was sliding sooo slow
            #to fix it :
            #hold brakes if proposed_linear_velocity too low while no brakes
            if (args[0] < 0.01) and (brake < self.brake_deadband):
                brake = self.brake_deadband
                #rospy.logdebug("========== HOLD BRAKE ========")

        return throttle, brake, steer
        
        
        
        
        
        
        
        
        