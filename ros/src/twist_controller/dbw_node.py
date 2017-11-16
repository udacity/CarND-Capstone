#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float64
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from lowpass import LowPassFilter
from twist_controller import TwistController
from yaw_controller import YawController
from pid import PID


'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.
You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.
One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.
We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.
We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.
Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.
'''
GAS_DENSITY = 2.858

PRINT_DEBUG = False # Print rospy.logwarn for debugging if True

class DBWNode(object):
    def __init__(self):
        # Init ros node dbw_node
        rospy.init_node('dbw_node')
        
        # Init class variables
        self.min_speed = 0;
        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        # Define publishers for the ros topics
        # Publisher for ros topic '/vehicle/steering_cmd'
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        
        # Publisher for ros topic '/vehicle/throttle_cmd'
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        
        # Publisher for ros topic '/vehicle/brake_cmd'
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)
                                         
                         
        # using YawController for steering and TwistController for throttle/brake
        self.controller_yaw = YawController(
            self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)     
        self.controller_twist = TwistController(
            self.vehicle_mass, self.fuel_capacity, self.brake_deadband, self.decel_limit, 
            self.accel_limit, self.wheel_radius, self.wheel_base, self.steer_ratio, self.max_lat_accel,
            self.max_steer_angle)        

        # Subscribe to the needed ros topics
        # Subsribe to ros topic '/current_velocity'
        rospy.Subscriber('/current_velocity', TwistStamped, callback = self.current_velocity_cb)
        
        # Subsribe to ros topic '/twist_cmd'
        rospy.Subscriber('/twist_cmd', TwistStamped, callback = self.twist_cmd_cb)
        
        # Subsribe to ros topic '/vehicle/dbw_enabled'
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, callback=self.dbw_enabled_cb)

        # Init members for storing content of subscriptions
        self.current_linear = 0.
        self.angular_velocity = 0.
        self.linear_velocity = 0.
        self.dbw_enabled = False
        
        # Init low pass filter for the steering command
        self.low_pass_filter = LowPassFilter(0.2,.1)
       
        # Time between two control loops
        self.elapsed_time = 0.    # why was it set to [],  its not a list but a scalar
        self.previous_time = 0.  
		
		#create a reset fall back solution
        self.dbw_reset()

        
        #declare everything above loop        
        self.loop()
  

    ### Begin: Callback and helper functions for subsribers to ROS topics
	
    def dbw_reset(self):
        self.controller_twist.reset()   #reset the controllers
        # Reseting all the variables of the class
        self.current_linear = 0.
        self.elapsed_time = 0.
        self.dbw_enabled = False
        # set previous time to now so that we can caculate elapsed time for PID when available
        self.previous_time = rospy.get_time()
        self.angular_velocity = 0.
        self.linear_velocity = 0.
 
    # Callback function to set the current velocity of the car
    # Information provided by ros topic '/current_velocity'
    def current_velocity_cb(self, msg):
        self.current_linear = msg.twist.linear.x
        self.current_angular = msg.twist.linear.z
        #rospy.logwarn('current linear vel %f',msg.twist.linear.x)

    # Callback function to get twist commands for the controller
    # Information provided by ros topic ''/twist_cmd''
    def twist_cmd_cb(self, msg):
        self.linear_velocity = msg.twist.linear.x
        self.angular_velocity = msg.twist.angular.z
        #rospy.logwarn('current linear and anglarvel %f  %f',msg.twist.linear.x, msg.twist.angular.z)

    # Callback function to get the information if the car should be controlled
    # by the control system or a safety driver takes control
    # Information provided by ros topic '/vehicle/dbw_enabled'
    def dbw_enabled_cb(self, msg):
        # Reset the controllers if safety driver takes control  
        # TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # TODO check what is necessary if control comes back
      #  if self.dbw_enabled == False and msg.data == True:
            # Reset the controllers
       #     self.controller_twist.reset()
        
        self.dbw_enabled = msg.data

    ### End: Callback functions for subsribers to ROS topics

    def loop(self):
        # Set update rate (50Hz)
        # TODO !!!!!!!!!!!!!!!!!!
        # TODO set to 50Hz for final solution
        # TODO set to 5Hz for testing issues
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # Get predicted throttle, brake, and steering
            # Only publish the control commands if dbw is enabled
            rospy.logwarn('DBW is %s \n\n\n', self.dbw_enabled)
            if self.dbw_enabled:
            # Get steering data from the control system
                steer = self.controller_yaw.get_steering(
                self.linear_velocity, 
                self.angular_velocity, 
                self.current_linear)
                			# Apply low pass to steering data
                steer = self.low_pass_filter.filt(steer)
                			# Update timing information
                current_time = rospy.get_time()
                self.elapsed_time = current_time - self.previous_time
                self.previous_time = current_time
                
                			# Get throttle/brake data from the control system
                throttle, brake = self.controller_twist.control(
                self.current_linear, self.linear_velocity,
                self.elapsed_time)
			# Calculate the final braking torque which has to be published
                brake_torque = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * brake * self.wheel_radius
					  
				# Only publish the commands for throttle, brake and steering
				# if the control system is enabled
                self.publish(throttle, brake_torque, steer)
            else:
                self.dbw_reset()
            
            rate.sleep()


    # Publish the commands for the drive-by-wire system to the ros topics
    # For steering: '/vehicle/steering_cmd'
    # For throttle: '/vehicle/throttle_cmd'
    # For braking: '/vehicle/brake_cmd'
    def publish(self, throttle, brake, steer):
        # Print some debug info
        if PRINT_DEBUG:
            rospy.logwarn('---------Commands throttle: %.3f, brake_torque: %.3f, steering: %.3f', throttle, brake, steer)
        
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
