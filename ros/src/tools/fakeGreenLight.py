#!/usr/bin/env python

import argparse
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from dbw_mkz_msgs.msg import BrakeCmd
import math
import sys
import numpy as np
import csv

MPS = 0.44704

class FakeGreenLight():
    def __init__(self):
        # initialize and subscribe to the current position and waypoint base topic
        rospy.init_node('fake_green_light')
        self.sub_current_velocity = rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        rospy.spin()

    def velocity_cb(self, msg):
        self.current_linear_velocity = msg.twist.linear.x
        self.current_angular_velocity = msg.twist.angular.z
        if self.current_linear_velocity > 9.*MPS:
            rospy.signal_shutdown("fake green light done.")
            sys.exit(0)
        else:
            self.upcoming_red_light_pub.publish(Int32(-1))


if __name__ == "__main__":
    try:
        FakeGreenLight()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not send fake green light message.')

