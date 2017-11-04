#!/usr/bin/env python

import argparse
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
import sys
import numpy as np
import csv

class DumpWaypoints():
    def __init__(self, outfile):
        # initialize and subscribe to the current position and waypoint base topic
        rospy.init_node('waypoint_grabber')
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb)
        self.pose = None
        self.waypoints = None
        self.outfile = outfile
        rospy.spin()

    def pose_cb(self, msg):
        """Grab the pose of the car

        Args:
           msg (PoseStamped): car pose with (x,y,z) and rotation (x,y,z,q)
        """
        self.pose = msg

    def waypoints_cb(self, msg):
        """Grab the first incoming camera image and saves it

        Args:
            msg (Image): image from car-mounted camera

        """
        self.waypoints = msg
        if self.pose:
            fieldname = ['x', 'y', 'z', 'ax', 'ay', 'az', 'aw']
            log_file = open(self.outfile, 'w')
            log_writer = csv.DictWriter(log_file, fieldnames=fieldname)
            log_writer.writeheader()
            for waypoint in msg.waypoints:
                log_writer.writerow({
                'x': waypoint.pose.pose.position.x,
                'y': waypoint.pose.pose.position.y,
                'z': waypoint.pose.pose.position.z,
                'ax': waypoint.pose.pose.orientation.x,
                'ay': waypoint.pose.pose.orientation.y,
                'az': waypoint.pose.pose.orientation.z,
                'aw': waypoint.pose.pose.orientation.w
                })
            log_file.close()
            rospy.signal_shutdown("dump waypoint done.")
            sys.exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Udacity SDC System Integration, Waypoints Dumper')
    parser.add_argument('outfile', type=str, help='Final Waypoint save file, first point is the starting point')
    args = parser.parse_args()

    try:
        DumpWaypoints(args.outfile)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not grab waypoints.')

