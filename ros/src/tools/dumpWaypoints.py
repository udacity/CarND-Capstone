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
        self.sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.pose = None
        self.waypoints = None
        self.outfile = outfile
        self.s = 0.
        self.ss = 0.
        rospy.spin()

    def pose_cb(self, msg):
        """Grab the pose of the car

        Args:
           msg (PoseStamped): car pose with (x,y,z) and rotation (x,y,z,q)
        """
        self.pose = msg
        self.sub1.unregister()
        self.sub1 = None

    def distance(self, waypoints, wp):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        wp0 = wp-1
        if wp0 < 0:
            wp0 = 0
        return dl(waypoints[wp0].pose.pose.position, waypoints[wp].pose.pose.position)

    def waypoints_cb(self, msg):
        """Grab the first incoming camera image and saves it

        Args:
            msg (Image): image from car-mounted camera

        """
        if self.pose is not None and self.waypoints is None:
            self.sub2.unregister()
            self.sub2 = None
            self.waypoints = msg.waypoints
            print "waypoints size: ", len(self.waypoints)
            print "pose: "
            print self.pose
            fieldname = ['x', 'y', 'z', 'ax', 'ay', 'az', 'aw', 's', 'ss']
            log_file = open(self.outfile, 'w')
            log_writer = csv.DictWriter(log_file, fieldnames=fieldname)
            log_writer.writeheader()
            log_writer.writerow({
                'x': self.pose.pose.position.x,
                'y': self.pose.pose.position.y,
                'z': self.pose.pose.position.z,
                'ax': self.pose.pose.orientation.x,
                'ay': self.pose.pose.orientation.y,
		'az': self.pose.pose.orientation.z,
                'aw': self.pose.pose.orientation.w,
                's': self.s,
                'ss': self.ss
                })
            for i in range(len(self.waypoints)):
                waypoint = self.waypoints[i]
                self.ss = self.distance(self.waypoints, i)
                self.s += self.ss
                log_writer.writerow({
                'x': waypoint.pose.pose.position.x,
                'y': waypoint.pose.pose.position.y,
                'z': waypoint.pose.pose.position.z,
                'ax': waypoint.pose.pose.orientation.x,
                'ay': waypoint.pose.pose.orientation.y,
		'az': waypoint.pose.pose.orientation.z,
                'aw': waypoint.pose.pose.orientation.w,
                's': self.s,
                'ss': self.ss
                })

            log_file.close()
            rospy.signal_shutdown("dump waypoint done.")
            sys.exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Udacity SDC System Integration, Waypoints Dumper')
    parser.add_argument('outfile', type=str, help='Waypoint save file, first point is the starting point')
    args = parser.parse_args()

    try:
        DumpWaypoints(args.outfile)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not grab waypoints.')

