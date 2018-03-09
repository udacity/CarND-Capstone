#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy import spatial
import math
import numpy as np

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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number

DEBUG_LEVEL = 1  # 0 no Messages, 1 Important Stuff, 2 Everything

UPDATE_FREQUENCY = 10  # 10Hz should do the trick :)

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.vel_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.current_pose = None
        self.next_wp_index = None
        self.kdtree = None
        self.last_traffic_wpi = -1
        self.current_vel = None
        self.brake_range = []

        # Get originally set velocity from the waypoint loader package
        self.orig_velocity = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity'))


        if DEBUG_LEVEL >= 1: rospy.logwarn("Waypoint Updater loaded!")
        rate = rospy.Rate(UPDATE_FREQUENCY)

        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

    def loop(self):
        if (self.current_pose is not None) and (self.base_waypoints is not None):
            self.next_wp_index = self.get_next_waypoint_index()
            self.publish_waypoints(self.next_wp_index)

    def pose_cb(self, msg):
        # When we get a new position then load it to the local variable
        self.current_pose = msg.pose

    def vel_cb(self, msg):
        self.current_vel = msg.twist.linear.x


    def waypoints_cb(self, waypoints):
        # When we the waypoints we save them to the local variable
        self.base_waypoints = waypoints.waypoints

    def kmph2mps(self, velocity_kmph):
        """ converts kmph to mps """
        return (velocity_kmph * 1000.) / (60. * 60.)

    def traffic_cb(self, msg):
        """
        Callback for the /traffic waypoint message
        Modifies the velocities of the waypoints in order to brake at a red traffic light

        Args:
            msg: message containing the waypoint of closest traffic light if it is red and 
            -1 otherwise 
        """

        if DEBUG_LEVEL >= 1: rospy.logwarn("traffic_cb - current wpi {0:d}, current speed {1:f}"
            .format(self.next_wp_index, self.current_vel))

        if (self.base_waypoints is not None):
            wpi = msg.data

            # Red traffic light ahead - check if brakes have to ba applied or not
            if (wpi != -1):
                dist = self.distance(self.base_waypoints, self.next_wp_index, wpi)
                brake_dist = (self.current_vel**2)/(2*2)
                index_dist = wpi - self.next_wp_index
                if (index_dist): 
                    s_per_idx = dist/index_dist
                else:
                    s_per_idx = 0.9
                
                if DEBUG_LEVEL >= 1: 
                    rospy.logwarn("closest stopline wp {0:d}, distance {1:f}".format(wpi, dist))
                    rospy.logwarn("brake distance {0:f}".format(brake_dist))
                    rospy.logwarn("s_per_idx {0:f}".format(s_per_idx))

                if (dist < brake_dist):
                    for i in range(self.next_wp_index, wpi):
                        v_new = self.current_vel - np.sqrt(2*3*s_per_idx*(i-self.next_wp_index))
                        if (v_new < 0.0):
                            v_new = 0.0
                        rospy.logwarn("i {0:d} - v_new {1:f}".format(i, v_new))
                        self.set_waypoint_velocity(self.base_waypoints, i, v_new)
                    self.brake_range.append((self.next_wp_index, wpi))
                    if DEBUG_LEVEL >= 1: rospy.logwarn("Brake between {0:d} and {1:d}".format(self.next_wp_index, wpi))
            
            # No red traffic light: reset velocities to the original value
            if (wpi == -1):
                for range_tuple in self.brake_range:
                    for i in range(range_tuple[0], range_tuple[1]):
                        self.set_waypoint_velocity(self.base_waypoints, i, self.orig_velocity)
                    if DEBUG_LEVEL >= 1: rospy.logwarn("Reset between {0:d} and {1:d}".format(range_tuple[0],range_tuple[1]))
                    self.brake_range.remove(range_tuple)

            """
            if(wpi != self.last_traffic_wpi):

                # If a traffic light turns green, reset veolcity values to orignal velaues
                if (self.last_traffic_wpi != -1):
                    for i in range(self.last_traffic_wpi-30, self.last_traffic_wpi+10):
                        self.set_waypoint_velocity(self.base_waypoints, i, self.orig_velocity)
                    if DEBUG_LEVEL >= 1: rospy.logwarn("Drive between {0:d} and {1:d}".format(self.last_traffic_wpi-25, self.last_traffic_wpi+10))
                
                # if a red traffic light is ahead, decrease velocity values linearly 
                # for each waypoint closer to the stop line
                # TODO: smarter braking - also account current speed and brake-distance
                if (wpi != -1):
                    for i in range(wpi-30, wpi-10):
                        wp_vel = abs(i-wpi+10)/20.0*self.orig_velocity
                        self.set_waypoint_velocity(self.base_waypoints, i, wp_vel)
                    for i in range(wpi-10, wpi-0):
                        wp_vel = 0.0
                        self.set_waypoint_velocity(self.base_waypoints, i, wp_vel)
                    for i in range(wpi+1, wpi+10):
                        wp_vel = abs(i-wpi)/10.0*self.orig_velocity
                        self.set_waypoint_velocity(self.base_waypoints, i, wp_vel)
                    if DEBUG_LEVEL >= 1: rospy.logwarn("Brake between {0:d} and {1:d}".format(wpi-25, wpi+10))
            self.last_traffic_wpi = wpi
            """


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    # Helper functions
    def get_next_waypoint_index(self):
        # prepare car position and orientation
        car_x = self.current_pose.position.x
        car_y = self.current_pose.position.y
        s = self.current_pose.orientation.w
        car_theta = 2 * np.arccos(s)
        # contain theta between pi and -pi
        if car_theta > np.pi:
            car_theta = -(2 * np.pi - car_theta)
        #check if kd-tree is loaded
        if self.kdtree == None:
            rospy.logwarn("Waypoint Updater - Load waypoints into k-d tree")
            wp_item_list = []
            for i, wp in enumerate(self.base_waypoints):
                x = wp.pose.pose.position.x
                y = wp.pose.pose.position.y
                wp_item_list.append([x, y])
            self.kdtree = spatial.KDTree(np.asarray(wp_item_list))
            if DEBUG_LEVEL >= 1:
                rospy.logwarn("Waypoint Updater - Begin sample waypoints in k-d tree")
                for i, wp in enumerate(self.base_waypoints):
                    x = wp.pose.pose.position.x
                    y = wp.pose.pose.position.y
                    dist, j = self.kdtree.query([x, y])
                    rospy.logwarn("Waypoint Updater wp: {0:d} pos: {1:.3f},{2:.3f}".format(i, x, y))
                    rospy.logwarn("Waypoint Updater kdtree: {0:d} pos: {1:.3f},{2:.3f}".format(j, self.base_waypoints[j].pose.pose.position.x, self.base_waypoints[j].pose.pose.position.y))
                    if i > 3: break
                rospy.logwarn("Waypoint Updater - End sample waypoints in k-d tree")
        dist, nwp_index = self.kdtree.query([self.current_pose.position.x, self.current_pose.position.y])
        nwp_x = self.base_waypoints[nwp_index].pose.pose.position.x
        nwp_y = self.base_waypoints[nwp_index].pose.pose.position.y
                
        # this will be the closest waypoint index without respect to heading
        heading = np.arctan2((nwp_y - car_y), (nwp_x - car_x))
        angle = abs(car_theta - heading);
        # so if the heading of the waypoint is over one quarter of pi its behind so take the next wp :)
        if (angle > np.pi / 4):
            nwp_index = (nwp_index + 1) % len(self.base_waypoints)

        return nwp_index

    def publish_waypoints(self, next_wp_index):
        msg = Lane()
        msg.waypoints = []
        index = next_wp_index
        for i in range(LOOKAHEAD_WPS):
            wp = Waypoint()
            wp.pose.pose.position.x = self.base_waypoints[index].pose.pose.position.x
            wp.pose.pose.position.y = self.base_waypoints[index].pose.pose.position.y
            wp.twist.twist.linear.x = self.base_waypoints[index].twist.twist.linear.x
            msg.waypoints.append(wp)
            index = (index + 1) % len(self.base_waypoints)
        if DEBUG_LEVEL >= 2: rospy.logwarn("Waypoints published! Next Waypoint x: {0:.3f} y: {1:.3f}".format(msg.waypoints[0].pose.pose.position.x, msg.waypoints[0].pose.pose.position.y))
        self.final_waypoints_pub.publish(msg)

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
