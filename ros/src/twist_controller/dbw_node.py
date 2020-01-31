#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
import math

from twist_controller import Controller

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

        # TODO: Create `Controller` object
        # self.controller = Controller(<Arguments you wish to provide>)
        #rospy.logwarn("[WARNING 1]\n")
        #rospy.logerr("[ERROR 1]\n")
        self.controller = Controller(vehicle_mass = vehicle_mass, fuel_capacity = fuel_capacity, 
        				brake_deadband = brake_deadband, decel_limit = decel_limit, 
        				accel_limit = accel_limit, wheel_radius = wheel_radius, 
        				wheel_base = wheel_base, steer_ratio = steer_ratio, 
        				max_lat_accel = max_lat_accel, max_steer_angle = max_steer_angle)

        # TODO: Subscribe to all the topics you need to
        self.proposed_lin_vel = 0.0
        self.proposed_ang_vel = 0.0
        self.current_lin_vel  = 0.0
        self.counter = 0
        
        # Subscribe to current velocity and intended linear/angular velocities
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_lin_vel_cb)
        
        # Subscribe to drive-by-wire
        self.dbw_is_enabled = False
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        
        # Subscribe to current pose for current angle
        self.pose = None
        self.pose_z = 0
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        self.loop()

    # Pose message callback to get angle for steering controller
    def pose_cb(self, msg):
        self.pose = msg
        self.pose_z = self.pose.pose.orientation.z
        
    # DBW message callback
    def dbw_enabled_cb(self, msg):
      # Get message and update dbw state
      self.dbw_is_enabled = msg.data

      #rospy.logwarn("[ LOG ] : Got a DBW state message = %s/n", self.dbw_is_enabled)
    # TwistStamped message callback on "/twist_cmd" topic
    # Get a TwistCmd message with fields:
    #   (*) <geometry_msgs/Twist> twist (http://docs.ros.org/api/geometry_msgs/html/Twist.html)
    #       This expresses velocity in free space broken down in linear and angular.
    #       (*) <Vector3> linear
    #           (*) <float64> x,y,z
    #       (*) <Vector3>  angular
    #           (*) <float64> x,y,z
    #   (*) <float32> accel_limit # m/s^2 where 0 is no limit
    #   (*) <float32> decel_limit # m/s^2 where 0 is no limit
    
    def twist_cmd_cb(self, msg):
      self.proposed_lin_vel = msg.twist.linear.x
      self.proposed_ang_vel = msg.twist.angular.z
                    
    # Callback on current speed message "/current_velocity" topic, message type TwistStamped, as <twist_cmd_cb()>
    def current_lin_vel_cb(self, msg):
      self.current_lin_vel = msg.twist.linear.x

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
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
            
            throttle, brake, steering = self.controller.control(self.proposed_lin_vel,
                                                                self.proposed_ang_vel,
                                                                self.current_lin_vel,
                                                                self.dbw_is_enabled)

            if (self.dbw_is_enabled == True):
            	self.publish(throttle, brake, steering)
            '''
            output_speed = 0.0
            if (self.proposed_lin_vel < 1.0):
              output_speed = 0.0
              brake = 1l
            else:
              output_speed = self.proposed_lin_vel
              brake = 0
              
            throttle = output_speed
            steering = self.proposed_ang_vel
            if (self.dbw_is_enabled == True):
              self.publish(throttle, brake, steering)l
            '''
            rate.sleep()

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
