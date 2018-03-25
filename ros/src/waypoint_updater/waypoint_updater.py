#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import tf
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

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):

        rospy.init_node('waypoint_updater')

        self.base_waypoints = None
        self.current_pose = None
        self.waypoints_ahead = None
        self.traffic_light_state = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.desired_vel = 0.0  # the desired vehicle velocity at each timestep
        self.max_vel = 11.0  # m/s
        self.ramp_dist = 35  # distance to ramp up and down the acceleration (m)
        self.acceleration = self.max_vel / self.ramp_dist

        self.updater_loop()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints

    def updater_loop(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():

            if self.current_pose and self.base_waypoints:
                # locate vehicle and update waypoints ahead
                ego_veh_wp = self.ego_veh_waypoint(self.current_pose, self.base_waypoints)
                self.waypoints_ahead = self.base_waypoints[ego_veh_wp:ego_veh_wp + LOOKAHEAD_WPS]

                # control vehicle speed depeding on traffic light state
                self.adaptive_cruise_control(self.base_waypoints, self.waypoints_ahead, ego_veh_wp, self.traffic_light_state)

                # publish
                final_waypoints_msg = Lane()
                final_waypoints_msg.header.stamp = rospy.Time.now()
                final_waypoints_msg.waypoints = self.waypoints_ahead
                self.final_waypoints_pub.publish(final_waypoints_msg)

            rate.sleep()

    def traffic_cb(self, msg):
        self.traffic_light_state = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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

    def ego_veh_waypoint(self, pose, waypoints):

        # find waypoint closest to vehicle's current pose
        dist_from_current_pose = 100000
        ego_veh_wp_idx = 0
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(len(waypoints)):
            dist = dl(pose.position, waypoints[i].pose.pose.position)
            if (dist < dist_from_current_pose):
                dist_from_current_pose = dist
                ego_veh_wp_idx = i

        # heading of the vehicle given current pose and next waypoint
        x_map = waypoints[ego_veh_wp_idx].pose.pose.position.x
        y_map = waypoints[ego_veh_wp_idx].pose.pose.position.y
        heading = math.atan2((x_map - pose.position.x), ((y_map - pose.position.y)))

        # convert from quaternion to euler coordinates
        pose_quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(pose_quaternion)

        # difference between heading angle (heading) and current angle based on velocity vector (yaw)
        angle = math.fabs(yaw - heading)

        # if angle is greater than 45 deg vehicle's current pose is behind the next waypoint
        if angle > math.pi / 4.0:
            ego_veh_wp_idx += 1

        return ego_veh_wp_idx

    def adaptive_cruise_control(self, waypoints, waypoints_ahead, waypoint, traffic_light_state):
        # simple cruise controller based on "distance" from traffic light and end of destination

        wheel_base = rospy.get_param('~wheel_base', 2.8498)

        if traffic_light_state > 0.0:
            dist_x = self.base_waypoints[traffic_light_state].pose.pose.position.x - \
                     self.base_waypoints[waypoint].pose.pose.position.x
            dist_y = self.base_waypoints[traffic_light_state].pose.pose.position.y - \
                     self.base_waypoints[waypoint].pose.pose.position.y

            # calculate the distance to target location from the front of the vehicle
            dist = math.sqrt(dist_x ** 2 + dist_y ** 2) - wheel_base

            if dist < 0.3:
                self.desired_vel = 0.
            elif dist < self.ramp_dist:  # ramp the velocity down when close to the stop point
                self.desired_vel = max(self.max_vel * dist / self.ramp_dist, 0.5)  # simple ramp function
        else:
            # ramp speed up with acceleration of 0.041(m/s)/ros_rate
            self.desired_vel = max(self.desired_vel + self.acceleration, 0.5)

            # stop the vehicle at the destination (first slow down vehicle, then stop)
            if len(waypoints_ahead) < 2: # stop at the last 2 waypoints
                self.desired_vel = 0.
            elif len(waypoints_ahead) < (LOOKAHEAD_WPS/4):  # ramp the velocity down at the last 25% of the LOOKAHEAD_WPS
                self.desired_vel = max(self.max_vel * len(waypoints_ahead) / (LOOKAHEAD_WPS/4), 0.5)  # simple ramp function

        vel = self.base_waypoints[waypoint].twist.twist.linear.x
        if vel > 0:
            self.desired_vel = min(self.desired_vel, vel)
        else:
            self.desired_vel = min(self.desired_vel, self.max_vel)
        self.set_waypoint_velocity(waypoints, waypoint, self.desired_vel)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')