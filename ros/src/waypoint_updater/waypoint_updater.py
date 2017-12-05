#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
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

def get_car_xy_from_global_xy(car_x, car_y, yaw_rad, global_x, global_y):
    # Translate global point by car's position
    xg_trans_c = global_x - car_x
    yg_trans_c = global_y - car_y
    # Perform rotation to finish mapping
    # from global coords to car coords
    x = xg_trans_c * math.cos(0 - yaw_rad) - yg_trans_c * math.sin(0 - yaw_rad)
    y = xg_trans_c * math.sin(0 - yaw_rad) + yg_trans_c * math.cos(0 - yaw_rad)
    return (x, y)

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.base_waypoints = None

        self.current_pose = None

        # Loop Rate
        self.loop_frequency = 50 # 50 Hz

        # Max velocity
        self.max_velocity = 10 # 10 mph, I guess....

        self.loop()

        rospy.spin()


    def loop(self):

        rate = rospy.Rate(self.loop_frequency)
        while not rospy.is_shutdown():
            if self.current_pose != None and self.base_waypoints != None:
                xyz_position = self.current_pose.position
                quaternion_orientation = self.current_pose.orientation

                p = xyz_position
                qo = quaternion_orientation

                p_list = [p.x, p.y, p.z]
                qo_list = [qo.x, qo.y, qo.z, qo.w]
                euler = euler_from_quaternion(qo_list)
                yaw_rad = euler[2]

                closest_waypoint_idx = None
                closest_waypoint_dist = None
                for idx in range(len(self.base_waypoints)):
                    waypoint = self.base_waypoints[idx]
                    wgx = waypoint.pose.pose.position.x
                    wgy = waypoint.pose.pose.position.y
                    wcx, wcy = get_car_xy_from_global_xy(p.x, p.y, yaw_rad, wgx, wgy)
                    if closest_waypoint_idx is None:
                        closest_waypoint_idx = idx
                        closest_waypoint_dist = math.sqrt(wcx**2 + wcy**2)
                    else:
                        curr_waypoint_dist = math.sqrt(wcx**2 + wcy**2)
                        if curr_waypoint_dist < closest_waypoint_dist:
                            closest_waypoint_idx = idx
                            closest_waypoint_dist = curr_waypoint_dist

                waypoint = self.base_waypoints[closest_waypoint_idx]
                wgx = waypoint.pose.pose.position.x
                wgy = waypoint.pose.pose.position.y
                wcx, wcy = get_car_xy_from_global_xy(p.x, p.y, yaw_rad, wgx, wgy)
                if wcx < 0:
                    closest_waypoint_idx = (closest_waypoint_idx + 1) % len(self.base_waypoints)

                next_waypoints = []
                for loop_idx in range(LOOKAHEAD_WPS):
                    wp_idx = (loop_idx + closest_waypoint_idx) % len(self.base_waypoints)
                    next_waypoints.append(self.get_waypoint_to_sent(wp_idx))

                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time(0)
                lane.waypoints = next_waypoints
                self.final_waypoints_pub.publish(lane)


            rate.sleep()

    def get_waypoint_to_sent(self, wp_idx):
        # changes will be here when the red lights are detected.
        # if the velocity is not set, 11 is set by default.
        self.set_waypoint_velocity(self.base_waypoints, wp_idx, self.max_velocity)
        return self.base_waypoints[wp_idx]


    def pose_cb(self, msg):
        self.current_pose = msg.pose

    def waypoints_cb(self, lane):
        self.base_waypoints = lane.waypoints

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
