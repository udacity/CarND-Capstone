#!/usr/bin/env python

import os
import csv
import math

from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray

from styx_msgs.msg import Lane, Waypoint

import tf
import rospy

CSV_HEADER = ['x', 'y', 'z', 'yaw']
MAX_DECEL = 1.0

def waypointToMarker(waypoint, frame_id, ts=rospy.Time(0), idx=0, color=[0.0, 1.0, 0.0]):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = ts
    marker.id = idx
    marker.pose.position = waypoint.pose.pose.position
    marker.pose.orientation = waypoint.pose.pose.orientation
    marker.scale.x = 5
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    return marker

class WaypointLoader(object):

    def __init__(self):
        rospy.init_node('waypoint_loader', log_level=rospy.DEBUG)

        self.pub = rospy.Publisher('/base_waypoints', Lane, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('/base_waypoints_markers', MarkerArray, queue_size=1, latch=True)

        self.velocity = self.kmph2mps(rospy.get_param('~velocity'))
        self.new_waypoint_loader(rospy.get_param('~path'))
        rospy.spin()

    def new_waypoint_loader(self, path):
        if os.path.isfile(path):
            waypoints = self.load_waypoints(path)
            self.publish(waypoints)
            self.publish_markers(waypoints)
            rospy.loginfo('Waypoints Loaded')
        else:
            rospy.logerr('%s is not a file', path)

    def quaternion_from_yaw(self, yaw):
        return tf.transformations.quaternion_from_euler(0., 0., yaw)

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def load_waypoints(self, fname):
        waypoints = []
        with open(fname) as wfile:
            reader = csv.DictReader(wfile, CSV_HEADER)
            for wp in reader:
                p = Waypoint()
                p.pose.pose.position.x = float(wp['x'])
                p.pose.pose.position.y = float(wp['y'])
                p.pose.pose.position.z = float(wp['z'])
                q = self.quaternion_from_yaw(float(wp['yaw']))
                p.pose.pose.orientation = Quaternion(*q)
                p.twist.twist.linear.x = float(self.velocity)

                waypoints.append(p)
        return self.decelerate(waypoints)

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def decelerate(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:-1][::-1]:
            dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.pub.publish(lane)

    def publish_markers(self, waypoints):
        color_grey = [0.8, 0.8, 0.8]
        array = MarkerArray()
        nth = 1
        max_waypoints = 500
        if len(waypoints) > max_waypoints:
            nth = int(len(waypoints) / max_waypoints)
        for i, waypoint in enumerate(waypoints):
            if i % nth == 0:
                marker = waypointToMarker(waypoint, '/world', idx=i, color=color_grey)
                array.markers.append(marker)
        self.marker_pub.publish(array)
        rospy.loginfo("Marking every {}th waypoint, for {} waypoints total...".format(nth, len(array.markers)))

if __name__ == '__main__':
    try:
        WaypointLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')
