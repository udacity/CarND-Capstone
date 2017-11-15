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
class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')
        min_speed = 0;
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
                                         
                         
        # using YawController for steering and 
        # TwistController for throttle/brake
        self.controller_yaw = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)		
        self.controller_twist = TwistController(
            vehicle_mass, fuel_capacity, brake_deadband, decel_limit, 
            accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel,
            max_steer_angle)        

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/current_velocity', TwistStamped, callback = self.current_velocity_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, callback = self.twist_cmd_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, callback=self.dbw_enabled_cb)
        #rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        #rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        #rospy.Subscriber('/cte', Float64, self.cte_cb, queue_size=1)

        # TODO: members for storing content of subscriptions
        self.current_linear = 0.
        self.angular_velocity = 0.
        self.linear_velocity = 0.
        self.dbw_enabled = False
        #self.cte = None
        
        # ToDo   Need to define a twist controller for the below code to work in a cleaner fashion
        # for first try every thing included  here
        #self.pid = PID(0.35,0.01,0.0,0.0,1.0)
        self.low_pass_filter = LowPassFilter(0.2,.1) # low pass for steering
        ####
       
       
        self.elapsed_time = []
        self.previous_time = 0.  
        
        
        #declare everything above loop        
        self.loop()
  

        #Callback functions
 
    def current_velocity_cb(self, msg):
        self.current_linear = msg.twist.linear.x
        self.current_angular = msg.twist.linear.z
        #rospy.logwarn('current linear vel %f',msg.twist.linear.x)

    def twist_cmd_cb(self, msg):
        self.linear_velocity = msg.twist.linear.x
        self.angular_velocity = msg.twist.angular.z
        #rospy.logwarn('current linear and anglarvel %f  %f',msg.twist.linear.x, msg.twist.angular.z)


    def dbw_enabled_cb(self, msg):
        
        if      self.dbw_enabled == False \
            and msg.data == True:

            self.controller_twist.reset()
        
        self.dbw_enable = msg.data


    def pose_cb(self, msg):
        self.pose = [msg.pose.position.x, msg.pose.position.y]
        #pass

    # def cte_cb(self, msg):
    #     self.cte = float(msg.data)
    #     #rospy.logwarn('DBWNode::cte_cb: %.3f', self.cte)

    def loop(self):

        rate = rospy.Rate(5) # 5 Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)
          
            #steered = self.low_pass_filter.filt(self.angular_velocity) 
            #steer = self.controller.get_steering(self.linear_velocity, steered, self.current_linear)
            #rospy.logwarn('Entering the publisher now  %s, %s',steer, self.dbw_enabled)        
            #if self.dbw_enabled:
            
            steer = self.controller_yaw.get_steering(
                self.linear_velocity, 
                self.angular_velocity, 
                self.current_linear)
            steer = self.low_pass_filter.filt(steer)          
                        

            current_time = rospy.get_time()
            self.elapsed_time = current_time - self.previous_time  
            self.previous_time = current_time            


            ###################################################################
            #rospy.logwarn('Entering the publisher now  %f',self.elapsed_time)
            #cte_v = self.linear_velocity - self.current_linear
            #throttle = self.pid.step (cte_v, self.elapsed_time)
            #self.publish(throttle, 0, steer)
            #rospy.logwarn('throttle=%.3f, steer=%.3f', step_err, steer)
            ###################################################################

            ###################################################################
            # if not self.cte is None:
            throttle, brake = self.controller_twist.control(
                self.current_linear, self.linear_velocity,
                self.elapsed_time)
                  
            self.publish(throttle, brake, steer)
            #else:
            #    rospy.logwarn('DBWNode::loop: no cte available yet!')
            ###################################################################
            
            rate.sleep()







#error = self.angular_z_proposed - self.angular_z_current



    def publish(self, throttle, brake, steer):
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
