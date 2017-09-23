#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
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

EPSILON_THROTTLE = 0.05
EPSILON_BRAKE = 0.05
EPSILON_STEER = 0.05

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
        #steer_ratio = rospy.get_param('~steer_ratio', 118.4)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Create `TwistController` object
        self.controller = Controller(vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                                     wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle)

        # Subscribe to all the topics you need to
        rospy.Subscriber('/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)

        # Members
        self.dbw_enabled = True
        self.current_velocity = None
        self.twist_cmd = None
        self.last_throttle = 0.0
        self.last_brake = 0.0
        self.last_steer = 0.0

        self.loop()

    def loop(self):
        #rate = rospy.Rate(50) # 50Hz
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)

            #rospy.logwarn("dwb_enabled={}, current_velocity={}, twist_cmd={}".format(
            #    self.dbw_enabled, self.current_velocity != None, self.twist_cmd != None
            #))

            if (self.dbw_enabled and self.current_velocity != None and self.twist_cmd != None):
                throttle, brake, steer = self.controller.control(self.twist_cmd, self.current_velocity)
                rospy.logwarn("Throttle={}, Brake={}, Steer={}, twist_angular_z={}".
                              format(throttle, brake, steer,
                                     self.twist_cmd.twist.angular.z))
                self.publish(throttle, brake, steer)

            rate.sleep()

    def dbw_enabled_cb(self, msg):
        rospy.loginfo("Received dbw_enabled message.")
        self.dbw_enabled = msg
        rospy.logdebug(msg)

    def current_velocity_cb(self, msg):
        rospy.loginfo("Received current_velocity message.")
        self.current_velocity = msg
        rospy.logdebug(msg)

    def twist_cmd_cb(self, msg):
        rospy.loginfo("Received twist_cmd message.")
        self.twist_cmd = msg
        rospy.logdebug(msg)

    def publish(self, throttle, brake, steer):
        if (abs(self.last_throttle - throttle) > EPSILON_THROTTLE):
            self.last_throttle = throttle
            tcmd = ThrottleCmd()
            tcmd.enable = True
            tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            tcmd.pedal_cmd = throttle
            self.throttle_pub.publish(tcmd)
            rospy.loginfo("Issued throttle command, value={}".format(throttle))
        else:
            rospy.loginfo("Did no issue throttle command, value={}, last value={}".format(throttle, self.last_throttle))

        if (abs(self.last_steer - steer) > EPSILON_STEER):
            self.last_steer = steer
            scmd = SteeringCmd()
            scmd.enable = True
            scmd.steering_wheel_angle_cmd = steer
            self.steer_pub.publish(scmd)
            rospy.loginfo("Issued steer command, value={}".format(steer))
        else:
            rospy.loginfo(
                "Did no issue steer command, value={}, last value={}".format(steer, self.last_steer))

        if (abs(self.last_brake - brake) > EPSILON_BRAKE):
            self.last_brake = brake
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
            bcmd.pedal_cmd = brake
            self.brake_pub.publish(bcmd)
            rospy.loginfo("Issued brake command, value={}".format(brake))
        else:
            rospy.loginfo(
                "Did no issue brake command, value={}, last value={}".format(brake, self.last_brake))

if __name__ == '__main__':
    DBWNode()
