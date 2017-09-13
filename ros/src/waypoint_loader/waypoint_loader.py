#!/usr/bin/env python

import os
import csv
import math
# import pickle

from geometry_msgs.msg import Quaternion

from styx_msgs.msg import Lane, Waypoint

import tf
import rospy

CSV_HEADER = ['x', 'y', 'z', 'yaw']
MAX_DECEL = 1.0


class WaypointLoader(object):

    def __init__(self):
        rospy.init_node('waypoint_loader', log_level=rospy.DEBUG)

        self.pub = rospy.Publisher('/base_waypoints', Lane, queue_size=1)

        self.velocity = rospy.get_param('~velocity')
        self.new_waypoint_loader(rospy.get_param('~path'))
        rospy.spin()

    def new_waypoint_loader(self, path):
        if os.path.isfile(path):
            waypoints = self.load_waypoints(path)
            self.publish(waypoints)
            rospy.loginfo('Waypoint Loded')
        else:
            rospy.logerr('%s is not a file', path)

    def quaternion_from_yaw(self, yaw):
        return tf.transformations.quaternion_from_euler(0., 0., yaw)

    def get_velocity(self, velocity):
        return velocity/3.6

    def load_waypoints(self, fname):
        waypoints = []
        # # Marcus temp code: waypoints from udacity track
        # with open('/home/merbar/git/Self-Driven/capstone_data/uda_sim_waypoints.pickle', 'rb') as fp:
        #     uda_sim_waypoints = pickle.load(fp)
        # for wp in uda_sim_waypoints:
        #     p = Waypoint()
        #     p.pose.pose.position.x = float(wp[0])
        #     p.pose.pose.position.y = float(wp[1])-5
        #     p.pose.pose.position.z = float(0.0)
        #     q = self.quaternion_from_yaw(0.0)
        #     p.pose.pose.orientation = Quaternion(*q)
        #     p.twist.twist.linear.x = float(6)

        #     waypoints.append(p)
        with open(fname) as wfile:
            reader = csv.DictReader(wfile, CSV_HEADER)
            for wp in reader:
                p = Waypoint()
                p.pose.pose.position.x = float(wp['x'])
                p.pose.pose.position.y = float(wp['y'])
                p.pose.pose.position.z = float(wp['z'])
                q = self.quaternion_from_yaw(float(wp['yaw']))
                p.pose.pose.orientation = Quaternion(*q)
                p.twist.twist.linear.x = float(self.velocity*0.27778)

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
            vel = math.sqrt(2 * MAX_DECEL * dist) * 3.6
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

    def publish(self, waypoints):
        # Rate reduced from 40 to 0.1 for large performance increase
        # Don't really see the need to continously send the same base waypoints at all?!
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)
            lane.waypoints = waypoints
            self.pub.publish(lane)
            rate.sleep()


if __name__ == '__main__':
    try:
        WaypointLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')
