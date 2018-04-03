#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Float32, Int32, Bool
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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
UPDATE_RATE = 5 # in Hz
MAX_PLANNED_DECEL = 2 # in m/s^2
STOP_AHEAD_WAYPOINT_IN_MTRS = 3.0


def distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def get_pos_for_pose(pose_stamped):
    return [pose_stamped.pose.position.x, pose_stamped.pose.position.y]


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # DONE: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Waypoint, self.obstacle_cb)

        self.final_waypoints_pub = []

        self.publish_final_waypoints = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.publish_hold_veh = rospy.Publisher('vehicle/hold_veh', Bool, queue_size=1)
        self.publish_dist_to_stop = rospy.Publisher('vehicle/dist_to_stop', Float32, queue_size=1)

        # DONE: Add other member variables you need below

        self.pose_stamped = PoseStamped()
        self.base_waypoints = None
        self.stop_id = -1
        self.next_i = None
        self.dist_to_stop = 1000000
        self.hold_veh = False

        self.loop()

    def loop(self):
        rate = rospy.Rate(UPDATE_RATE)
        while not rospy.is_shutdown():
            if self.base_waypoints is not None and self.pose_stamped is not None:
                self.set_output_waypoints()
                self.set_output_velocities()
                self.publish_waypoints()
                self.publish_hold_veh.publish(self.hold_veh)
                self.publish_dist_to_stop.publish(self.dist_to_stop)
            rate.sleep()
        rospy.spin()

    def calc_yaw(self):
        # Calculate theta from quaternions as described in
        # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        current_orient = [self.pose_stamped.pose.orientation.x,
                          self.pose_stamped.pose.orientation.y,
                          self.pose_stamped.pose.orientation.z,
                          self.pose_stamped.pose.orientation.w]
        euler_vec = tf.transformations.euler_from_quaternion(current_orient)
        return euler_vec[2]

    def set_output_waypoints(self):
        theta = self.calc_yaw()
        self.find_next_wp(theta,get_pos_for_pose(self.pose_stamped))

        output_waypoints = []
        for i in range(LOOKAHEAD_WPS):
            wp_index = (i + self.next_i) % len(self.base_waypoints)

            # We want to modify the velocity property of the waypoints,
            # but not for the base waypoints, thats why we need to use
            # deepcopy!
            next_wp = copy.deepcopy(self.base_waypoints[wp_index])
            output_waypoints.append(next_wp)

        self.final_waypoints_pub = output_waypoints

    def set_output_velocities(self):

        if self.stop_id > 0:
            stop_id_in_final_wps = self.stop_id - self.next_i

            if stop_id_in_final_wps < len(self.final_waypoints_pub):

                self.dist_to_stop = distance(get_pos_for_pose(self.base_waypoints[self.next_i].pose),
                                             get_pos_for_pose(self.final_waypoints_pub[stop_id_in_final_wps].pose)
                                             )

                if self.dist_to_stop < 10:
                    self.hold_veh = True

                # For the waypoints backwards, set the velocity so that it decreases towards the stopping point
                for j in range(len(self.final_waypoints_pub)):

                    if j < stop_id_in_final_wps:
                        # Calculate the distance of the stopping point
                        dist_to_stop_point = distance(
                            get_pos_for_pose(self.final_waypoints_pub[j].pose),
                            get_pos_for_pose(self.final_waypoints_pub[stop_id_in_final_wps].pose))

                        # Calculate the maximum velocity for the waypoint
                        # in order to comfortably brake till the stopping point
                        max_vel_for_comf_decel = math.sqrt(max(0.0, 2.0 * MAX_PLANNED_DECEL * (dist_to_stop_point-STOP_AHEAD_WAYPOINT_IN_MTRS)))

                        # Get the velocity originally set for that waypoint
                        orig_vel = self.get_waypoint_velocity(self.final_waypoints_pub[j])

                        # Set the velocity to the minimum of both
                        self.set_waypoint_velocity(self.final_waypoints_pub,j,
                                                   min(max_vel_for_comf_decel,orig_vel))

                    else:

                        # Set the target velocity for the respective and all following waypoints to zero
                        self.set_waypoint_velocity(self.final_waypoints_pub, j, 0.)

            else:

                rospy.loginfo('The traffic light detector published a traffic waypoint that is outside the LOOKAHEAD_WPS')

        else:

            self.dist_to_stop = 1000000
            self.hold_veh = False

    def publish_waypoints(self):
        lane_pub_out = Lane()
        lane_pub_out.header.stamp = rospy.Time(0)
        lane_pub_out.waypoints = self.final_waypoints_pub
        self.publish_final_waypoints.publish(lane_pub_out)

    def find_next_wp(self,theta,pos):
        self.next_i = self.closest_wp(pos)

        # Get x and y for closest waypoint
        next_pos = get_pos_for_pose(self.base_waypoints[self.next_i].pose)
        next_x = next_pos[0]
        next_y = next_pos[1]

        # Check if it is in front of the position
        car_direction = math.atan2(next_y - pos[1], next_x - pos[0])
        waypoint_rel_direction = math.fabs(theta - car_direction)

        if math.pi/2 < waypoint_rel_direction < math.pi * 1.5:
            self.next_i = self.next_i + 1

        self.next_i = self.next_i % len(self.base_waypoints)

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
        # DONE: Callback for /traffic_waypoint message. Implement
        self.stop_id = int(msg.data)
        rospy.loginfo_throttle(1, 'Traffic callback done:{}'.format(self.stop_id))


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
