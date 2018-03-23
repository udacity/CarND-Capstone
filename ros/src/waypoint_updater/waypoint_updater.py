#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import tf
import copy

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
UPDATE_RATE = 5 # in Hz


def distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def get_pos_for_pose(pose_stamped):
    return [pose_stamped.pose.position.x, pose_stamped.pose.position.y]


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = []

        self.publish_final_waypoints = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.pose_stamped = PoseStamped()
        self.base_waypoints = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(UPDATE_RATE)
        while not rospy.is_shutdown():
            if self.base_waypoints is not None and self.pose_stamped is not None:
                self.set_output_waypoints()
                self.set_output_velocities()
                self.publish_waypoints()
            rate.sleep()
        rospy.spin()

    def calc_yaw(self):
        # Calculate theta from quaternions as described in https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        current_orient = [self.pose_stamped.pose.orientation.x,
                          self.pose_stamped.pose.orientation.y,
                          self.pose_stamped.pose.orientation.z,
                          self.pose_stamped.pose.orientation.w]
        euler_vec = tf.transformations.euler_from_quaternion(current_orient)
        return euler_vec[2]

    def set_output_waypoints(self):
        theta = self.calc_yaw()
        next_waypoint = self.find_next_wp(theta,get_pos_for_pose(self.pose_stamped))

        output_waypoints = []
        for i in range(LOOKAHEAD_WPS):
            wp_index = (i + next_waypoint) % len(self.base_waypoints)

            # We want to modify the velocity property of the waypoints,
            # but not for the base waypoints, thats why we need to use
            # deepcopy!
            next_wp = copy.deepcopy(self.base_waypoints[wp_index])
            output_waypoints.append(next_wp)

        self.final_waypoints_pub = output_waypoints

    def set_output_velocities(self):
        # For now, just set all the velocities to 10 m/s
        fixed_speed = 10.0

        for i in range(len(self.final_waypoints_pub)):
            self.set_waypoint_velocity(self.final_waypoints_pub,i,fixed_speed)

    def publish_waypoints(self):
        lane_pub_out = Lane()
        lane_pub_out.header.stamp = rospy.Time(0)
        lane_pub_out.waypoints = self.final_waypoints_pub
        self.publish_final_waypoints.publish(lane_pub_out)


    def find_next_wp(self,theta,pos):
        next_i = self.closest_wp(pos)

        # Get x and y for closest waypoint
        next_pos = get_pos_for_pose(self.base_waypoints[next_i].pose)
        next_x = next_pos[0]
        next_y = next_pos[1]

        # Check if it is in front of the position
        car_direction = math.atan2(next_y - pos[1], next_x - pos[0])
        waypoint_rel_direction = math.fabs(theta - car_direction)

        if math.pi/2 < waypoint_rel_direction < math.pi * 1.5:
            next_i = next_i + 1

        next_i = next_i % len(self.base_waypoints)

        return next_i


    def closest_wp(self, pos):
        best_dist = 1000000000
        best_i = 0

        for i in range(len(self.base_waypoints)):
            i_dist = distance(pos,get_pos_for_pose(self.base_waypoints[i].pose))
            if i_dist < best_dist:
                best_i = i
                best_dist = i_dist
        return best_i

    def pose_cb(self, msg):
        # DONE: Implement
        self.pose_stamped = msg

    def waypoints_cb(self, msg):
        # DONE: Implement
        self.base_waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, ind, velocity):
        waypoints[ind].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
