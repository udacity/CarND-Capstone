#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float32
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller
from yaw_controller import YawController
from lowpass import LowPassFilter
import pid

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
def twist_to_xyy(msg):
    x = msg.twist.linear.x
    y = msg.twist.linear.y
    yaw = msg.twist.angular.z
    return(x,y,yaw)


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')
        rospy.loginfo("initing dbw_node")

        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
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

        # self.prev_steer_val = None
        # self.prev_steer_val_set = False
        # self.tl_distance = -1
        # self.prev_tl_distance = -1
        # self.red_tl = True
        # self.tl_count = 0

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/tl_distance', Float32, self.tl_distance_cb)
        rospy.Subscriber('/go_to_stop_state', Bool, self.go_to_stop_cb)

        self.dbw = False
        self.angular_velocity_filter = LowPassFilter(.90, 1)
        self.velocity_filter = LowPassFilter(.9, 1)
        self.twist_yaw_filter = LowPassFilter(.2, .96)
        self.twist_velocity_filter = LowPassFilter(.96, .9)
        self.steer_filter = LowPassFilter(.2, .90)
        # self.p_v = [1.187355162, 0.044831144, 0.00295747] # v1.3
        self.p_v = [1.325735147117472, 0.06556341512981727, 0.013549012506233077] #  [1.9285529383307387, 0.0007904838169666957, 0.019058015342866958]
        self.pidv = pid.PID(self.p_v[0], self.p_v[1], self.p_v[2])
        self.throttle = 0.
        min_speed = .01
        # At high speeds, a multiple of 1.2 seems to work a bit
        # better than 1.0
        self.yaw_controller = YawController(wheel_base, 1.2*steer_ratio, min_speed, 8*max_lat_accel, max_steer_angle)
        # contol if the car is going to stop
        self.go_to_stop = True

        self.loop()

    '''
    /twist_cmd
    Type: geometry_msgs/TwistStamped
    geometry_msgs/TwistStampedstd_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Twist twist
      geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z
    '''

    def go_to_stop_cb(self, msg):
        self.go_to_stop = msg.data

    def tl_distance_cb(self, msg):
        # self.tl_distance = msg.data
        # if self.prev_tl_distance == msg.data == -1:
        #     self.tl_count += 1 % 1000
        #     if self.tl_count > 10: self.red_tl = False
        # else:
        #     self.tl_count = 0
        #     self.prev_tl_distance = msg.data
        #     self.red_tl = True
        pass


    def twist_cmd_cb(self, msg):
        seq = msg.header.seq
        (x, y, yaw) = twist_to_xyy(msg)
        # rospy.loginfo("twist_cmd_cb %d", seq)
        # print("twist linear: [%f, %f, %f]" % (x, y, yaw))
        self.twist_yaw_filter.filt(yaw)
        # if self.red_tl == True:
        #     vtwist = self.twist_velocity_filter.filt(0.1)
        # else:
        vtwist = self.twist_velocity_filter.filt(x)
        # calculate error between desired velocity and current velocity
        e = vtwist - self.velocity_filter.get()
        # feed pid controller with a dt of 0.033
        self.throttle = self.pidv.step(e, 0.033)
        if msg.header.seq%5 == 0:
            ts = msg.header.stamp.secs + 1.e-9*msg.header.stamp.nsecs
            # rospy.loginfo("tcc %d %f %f  %f %f", seq, ts, x, yaw, self.twist_yaw_filter.get())
        pass

    '''
    /vehicle/dbw_enabled
    Type: std_msgs/Bool
    '''
    def dbw_enabled_cb(self, msg):
        rospy.loginfo("dbw_enabled_cb %s", msg.data)
        self.dbw = msg.data

    '''
    /current_velocity
    Type: geometry_msgs/TwistStamped
    '''
    def current_velocity_cb(self, msg):
        seq = msg.header.seq
        (x, y, yaw) = twist_to_xyy(msg)
        # rospy.loginfo("current_velocity_cb %i", seq)
        # factor = .90
        # if yaw != 0.:
        # self.filtered_angular_velocity = factor*self.filtered_angular_velocity + (1-factor)*yaw
        # self.filtered_velocity = factor*self.filtered_velocity + (1-factor)*x
        self.angular_velocity_filter.filt(yaw)
        self.velocity_filter.filt(x)

        if msg.header.seq%5 == 0:
            ts = msg.header.stamp.secs + 1.e-9*msg.header.stamp.nsecs
            # ts seems to always be 0.
            yaw_per_meter = self.angular_velocity_filter.get()/self.velocity_filter.get()
            # rospy.loginfo("cv %d  %f %f  %f %f  %f", seq, self.velocity_filter.get(), x, self.angular_velocity_filter.get(), yaw, yaw_per_meter)
        pass


    def loop(self):
        # rate = rospy.Rate(50) # 50Hz
        rate = rospy.Rate(10) # 10Hz
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
            if self.dbw == True:
                twist = self.twist_yaw_filter.get()
                #steer = self.yaw_controller.get_steering(self.twist_velocity_filter.get(), twist, self.velocity_filter.get())
                steer = self.yaw_controller.get_steering(self.velocity_filter.get(), twist, self.velocity_filter.get())
                steer = self.steer_filter.filt(steer)


                # define throttle command based on pid controller
                throttle = brake = 0.

                if self.go_to_stop == False:

                    if abs(self.throttle) > 1.:
                        if self.throttle > 0: 
                            throttle = 1.0
                        else: 
                            throttle = 0.
                            brake = 1.0 * 1000
                    elif self.throttle < 0:
                        throttle = 0
                        brake = (abs(self.throttle) + self.brake_deadband) * 1000
                        if brake > 1.: brake = 1. * 1000 
                    else:
                        throttle = self.throttle + self.brake_deadband
                        if throttle > 1. : throttle = 1.
                else:
                    v = self.velocity_filter.get()
                    t = 1.0  # time to brake
                    u = 0.7  # friction coeficcient
                    g = 9.8  # gravity force 
                    D = (v * t) + (v**2 / (2. * u * 9.8))
                    # calculate work needed to stop
                    w = 0.5 * self.vehicle_mass * v**2
                    # caculate the force
                    F = w / D
                    brake = (2 * u * F * self.wheel_radius) + self.brake_deadband
                    brake *= 100
                    rospy.loginfo("throttle: %f brake: %f steering angle: %f " % (throttle, brake , steer))

                # throttle is 0.35, which runs the car at about 40 mph.
                # throttle of 0.98 will run the car at about 115 mph.
                self.publish(throttle, brake, steer)
            else:
                self.pidv.reset()
                
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
