#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math

import numpy as np
from scipy.interpolate import CubicSpline

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

        # TODO: Create `TwistController` object
        args = {
            'vehicle_mass': vehicle_mass,
            'fuel_capacity': fuel_capacity,
            'brake_deadband': brake_deadband,
            'decel_limit': decel_limit,
            'accel_limit': accel_limit,
            'wheel_radius': wheel_radius,
            'wheel_base': wheel_base,
            'steer_ratio': steer_ratio,
            'max_lat_accel': max_lat_accel,
            'max_steer_angle': max_steer_angle
        }

        self.controller = Controller(**args)

        # Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbwEnabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.currvelocity_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twistcmd_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # other member variables
        self.my_dbwEnabled = True
        self.my_current_velocity = None
        self.my_twist_command = None
        self.pose = None
        self.waypoints = None

        # start loop
        self.loop()

    def dbwEnabled_cb(self,dbwEnb):
        self.my_dbwEnabled = dbwEnb

    def currvelocity_cb(self,velocity):
        self.my_current_velocity = velocity

    def twistcmd_cb(self,twistcmd):
        self.my_twist_command = twistcmd

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def loop(self):
        rate = rospy.Rate(10) # 50Hz
        while not rospy.is_shutdown():

            if ((self.my_twist_command is not None) and
                (self.my_current_velocity is not None) and
                (self.pose is not None) and
                (self.waypoints is not None)):
                set_linear_velocity = self.my_twist_command.twist.linear.x
                set_angular_velocity = self.my_twist_command.twist.angular.z
                if (self.my_current_velocity is not None):
                    set_curr_velocity = self.my_current_velocity.twist.linear.x
                else:
                    set_curr_velocity = 0.0

                # cross-track error
                cte = self.calc_cte()
                dt = 0.02 #rospy rate

                throttle, brake, steering = self.controller.control( cte, dt, set_linear_velocity, set_angular_velocity, set_curr_velocity)
                # throttle, brake, steering = self.controller.control(self.my_current_velocity)
                if (self.my_dbwEnabled==True):
                    #steering = set_angular_velocity * 180/math.pi
                    print 'cte', cte, 'throttle', throttle, 'brake', brake, 'steer', steering, 'currspeed', set_curr_velocity, 'setspeed', set_linear_velocity
                    self.publish(throttle, brake, steering)

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

    def calc_cte(self):
        cte = 0.0
        if (self.waypoints is not None):
            closestWPi = self.get_closest_waypoint()
            interp_x = [
                self.waypoints.waypoints[closestWPi-1].pose.pose.position.x,
                self.waypoints.waypoints[closestWPi].pose.pose.position.x,
                self.waypoints.waypoints[closestWPi+1].pose.pose.position.x ]
            interp_y = [
                self.waypoints.waypoints[closestWPi-1].pose.pose.position.y,
                self.waypoints.waypoints[closestWPi].pose.pose.position.y,
                self.waypoints.waypoints[closestWPi+1].pose.pose.position.y ]
            cs = CubicSpline(interp_x, interp_y)
            t = np.linspace(interp_x[0], interp_x[2], 100)
            best_dist = 9999.99
            for i in range(len(t)):
                this_dist = math.sqrt( (self.pose.pose.position.x-t[i])**2 +
                                        (self.pose.pose.position.y-cs(t[i]))**2 +
                                        (self.pose.pose.position.z-0.0)**2 )
                if (this_dist<best_dist):
                    best_dist = this_dist
            cte = best_dist

        if (cte != 0.0):
            # sign of cte
            angle1 = math.atan2(interp_y[1]-interp_y[0],interp_x[1]-interp_x[0])
            angle2 = math.atan2(interp_y[2]-interp_y[0],interp_x[2]-interp_x[0])
            angle3 = math.atan2(self.pose.pose.position.y-interp_y[0],self.pose.pose.position.x-interp_x[0])
            # simplified case for now (ignore sharp turns, and discard middle point):
            if (angle3<angle2):
                cte = -cte
            else:
                cte = cte

        return cte

    def get_closest_waypoint(self):
        """Identifies the closest path waypoint to the current pose position
        Returns: int: index of the closest waypoint in self.waypoints
        """
        best_dist = 9999.99
        best_i = -1
        if (self.pose is not None):
            dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
            for i in range(len(self.waypoints.waypoints)):
                this_dist = dl(self.pose.pose.position,self.waypoints.waypoints[i].pose.pose.position)
                if (this_dist<best_dist):
                    best_dist = this_dist
                    best_i = i
        else:
            rospy.logerr('Could not find current pose.')
        return best_i

if __name__ == '__main__':
    DBWNode()
