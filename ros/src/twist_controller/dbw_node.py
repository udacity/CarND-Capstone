#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller
from yaw_controller import YawController

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

class DBWNode(object):
    
    dbw_enabled = False
    target_twist = None
    current_velocity = None
    prev_throttle = 0
    prev_brake = 0
    
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)
        self.yaw_controller = YawController(wheel_base,steer_ratio,0.0,max_lat_accel,max_steer_angle)
        # TODO Pass in proper initialization values to the controller
        self.controller = Controller()

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        
        self.loop()
        
    def current_velocity_cb(self, cv):
        '''
        cv is a TwistStamped command with values in meters per second
        for reference:
        100MPH = ~44 m/s
        50MPH = ~22 m/s
        '''
        
        self.current_velocity = cv.twist
        #self.current_velocity.linear.x *= 0.44704 # convert to m/s from mph
        rospy.loginfo('@_1 Curr x %s y %s', str(self.current_velocity.linear.x), str(self.current_velocity.linear.y))

    
    def dbw_enabled_cb(self, dbw):
        self.dbw_enabled = dbw
        
    def twist_cb(self, twistCommand):
        '''
        Pull in the TwistCommand
        '''
        self.target_twist = twistCommand.twist
        rospy.loginfo('@_1 Target x %s z %s', str(twistCommand.twist.linear.x), str(twistCommand.twist.angular.z))

        pass

    def loop(self):
        rate = rospy.Rate(60) # NHz
        while not rospy.is_shutdown():
            # Return if there is no target velocity to meet
            if self.target_twist and self.current_velocity:

                brake_error = 0
                
                if(self.current_velocity.linear.x - self.target_twist.linear.x) > 2:
                    # Normalise the brake error as a fraction of (from ~25 mph)
                    # meaning that 25 mph overspeed is maximum we are aiming for
                    brake_error = (self.current_velocity.linear.x - self.target_twist.linear.x)/10;
                    
                rospy.loginfo('@_1 Computing PID vel_err %s & brk_err %s',str((self.target_twist.linear.x  - self.current_velocity.linear.x)/44.704), str(brake_error))
                
                # Pass in the normalized velocity error to the controller
                t, b, s = self.controller.control(error_velocity = (self.target_twist.linear.x  - self.current_velocity.linear.x)/44.704, error_brake = brake_error)
            
                if not self.dbw_enabled:
                    self.controller.reset()
            
                if brake_error == 0:
                    b = 0
                b *= 10000
                s = self.yaw_controller.get_steering(linear_velocity=self.target_twist.linear.x, 
                                    angular_velocity=self.target_twist.angular.z, 
                                    current_velocity=self.current_velocity.linear.x)
            
                rospy.loginfo('@_1 Computing PID %s brake %s YAW %s',str(t), str(b), str(s))
                # TODO: Get predicted throttle, brake, and steering using `twist_controller`
                # You should only publish the control commands if dbw is enabled
                # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
                #                                                     <proposed angular velocity>,
                #                                                     <current linear velocity>,
                #                                                     <dbw status>,
                #                                                     <any other argument you need>)
                #if  self.dbw_enabled:
                self.publish(t,b,s)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        # Only publish with updated values. This is to
        # fix the lag issue
        if brake > 0:
            if brake != self.prev_brake:
                self.prev_brake = brake
                bcmd = BrakeCmd()
                bcmd.enable = True
                bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
                bcmd.pedal_cmd = brake
                self.brake_pub.publish(bcmd)
        else:
            #if throttle != self.prev_throttle:
            self.prev_throttle = throttle
            tcmd = ThrottleCmd()
            tcmd.enable = True
            tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            tcmd.pedal_cmd = throttle
            self.throttle_pub.publish(tcmd)
            self.prev_throttle = throttle

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

if __name__ == '__main__':
    DBWNode()
