#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        self.prev_nrst_wp = 0 # total number of waypoints are 10902

        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # new_wp_lane = Lane()
        # rospy.logwarn("attributes of lane class %s", new_wp_lane.__dict__)

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        # current pose of the vehicle
        self.vehicle_pos = msg.pose.position

        # vehicle orientation in quanternions
        self.vehicle_orientation = msg.pose.orientation
        # rospy.logwarn("current position of vehicle: %s", msg.pose.position.x)

    def waypoints_cb(self, waypoints):

        # TODO: Implement
        try: # to catch when the error when self.vehicle_pos has not been created yet
            smallest_dist = float('inf')
            nearest_wp = 0

            # rospy.logwarn("previous nearest waypoint: %s", self.prev_nrst_wp)

            self.wp_num = len(waypoints.waypoints)
            dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
            hd = lambda a, b: math.atan2((b.y-a.y), (b.x-a.x))
            

            for i in xrange(self.prev_nrst_wp, self.wp_num):
                
                wp_pos = waypoints.waypoints[i].pose.pose.position

                # distance between vehichle and the nearest waypoint
                dist = dl(self.vehicle_pos, wp_pos)

                if dist < smallest_dist:
                    nearest_wp = i
                    smallest_dist = dist

            # # quaternion conversion (see: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion)
            # q = self.vehicle_orientation
            # theta = math.asin(2*(q.w*q.y + q.z*q.x))
            # # if abs(theta)>= 1:
            # #     theta = math.pi/2.0
            # # else:
            # #     theta = math.asin(theta)
            # heading =  hd(self.vehicle_pos, wp_pos)
            # angle = abs(theta - heading)

            # if (angle>math.pi/4.0):
            #     nearest_wp += 1

            final_wps = self.get_final_wps(waypoints, nearest_wp)
            self.prev_nrst_wp = nearest_wp
            if nearest_wp > 10696:
                self.prev_nrst_wp = 0
            rospy.logwarn("nearest waypoint: %s", nearest_wp)

        except AttributeError, e:
            # rospy.logwarn("Error: %s", e) # optional: print out error
            pass
        else:
            # publish final waypoints only if vehicle position has been received
            self.final_waypoints_pub.publish(final_wps)


    def get_final_wps(self, waypoints, nearest_wp):
        new_wp_lane = Lane()
        for i in xrange(nearest_wp,nearest_wp+LOOKAHEAD_WPS):
            if i >= self.wp_num:
                break
            new_wp_lane.waypoints.append(waypoints.waypoints[i])
        return new_wp_lane

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

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
