#!/usr/bin/env python

import argparse
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane, Waypoint
import math
import sys
import numpy as np
import csv

class DumpCurrentPos():
    def __init__(self, outfile):
        # initialize and subscribe to the current twist command topic
        rospy.init_node('current_twist_dump')
        sub1 = rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        self.pose = None
        self.outfile = outfile
        self.fieldname = ['x', 'y', 'z', 'ax', 'ay', 'az', 'aw']
        self.log_file = open(self.outfile, 'w')
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=self.fieldname)
        self.log_writer.writeheader()
        rospy.spin()

    def twist_cb(self, msg):
        """Grab the current twist cmd of the car

        Args:
           msg (PoseStamped): car pose with (x,y,z) and rotation (x,y,z,q)
        """
        self.twist = msg
        self.log_writer.writerow({
            'x': self.twist.twist.linear.x,
            'y': self.twist.twist.linear.y,
            'z': self.twist.twist.linear.z,
            'ax': self.twist.twist.angular.x,
            'ay': self.twist.twist.angular.y,
            'az': self.twist.twist.angular.z
            })
        self.log_file.flush()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Udacity SDC System Integration, Waypoints Dumper')
    parser.add_argument('outfile', type=str, help='current pose save file, first point is the starting point')
    args = parser.parse_args()

    try:
        DumpCurrentPos(args.outfile)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not dump current pose.')

