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
LOOKAHEAD_TIME_THRESHOLD = 5 # seconds
NORMAL_SPEED = 10             # the normal speed of the car

import tf                       # This is of ROS geometry, not of TensorFlow!
def get_yaw(orientation):
    """
    Compute yaw from orientation, which is in Quaternion.
    """
    # orientation = msg.pose.orientation
    euler = tf.transformations.euler_from_quaternion([
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w])
    yaw = euler[2]
    return yaw
def to_local_coordinates(local_origin_x, local_origin_y, rotation, x, y):
    """
    compute the local coordinates for the global x, y coordinates values,
    given the local_origin_x, local_origin_y, and the rotation of the local x-axis.
    Assume the rotation is radius
    """
    shift_x = x - local_origin_x
    shift_y = y - local_origin_y

    cos_rotation = math.cos(rotation)
    sin_rotation = math.sin(rotation)

    local_x = cos_rotation*shift_x + sin_rotation*shift_y
    local_y = sin_rotation*shift_x + cos_rotation*shift_y

    return local_x, local_y
def publish_Lane(publisher, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        publisher.publish(lane)

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None  # indicating the base_waypoints is not yet available

        rospy.spin()

    import math
    
    def pose_cb(self, msg):
        # WORKING: Implement
        #
        if self.base_waypoints is None:
            return                  # no point of continue
        # end of if not self.base_waypoints_availble
        current_pose = msg.pose.position
        current_orientation = msg.pose.orientation
    
        # Compute the waypoints ahead of the current_pose
        waypoints_ahead = []
        waypoints_count = 0
        lookahead_dist = 0  # the accumulated distance of the looking ahead
        lookahead_time = 0  # the lookahead time
        prev_waypoint_pose = current_pose
    
        # the waypoints should be continuous
        # assume the base_waypoints are consecutive
        # the waypoints ahead should be continuous once started
    
        # waypoint_continued = True #TBD
    
        for waypoint in self.base_waypoints.waypoints:
            w_pos = waypoint.pose.pose.position
            yaw = get_yaw(current_orientation)
            local_x, local_y = to_local_coordinates(current_pose.x, current_pose.y, yaw,
                                                    w_pos.x, w_pos.y)
            if (0 < local_x) and (math.atan2(local_y, local_x) < math.pi/3):
                # the angle from my_car's orientation is less than 60 degree
                waypoints_ahead.append((waypoint, local_x, local_y))
                waypoints_count += 1
                dist_between = math.sqrt((prev_waypoint_pose.x-w_pos.x)**2 +
                                         (prev_waypoint_pose.y-w_pos.y)**2  +
                                         (prev_waypoint_pose.z-w_pos.z)**2)
                lookahead_dist += dist_between
                lookahead_time = lookahead_dist / (NORMAL_SPEED)
                prev_waypoint_pose = w_pos
                # waypoint_found = True # TBD
            else:
                # waypoint_found = ?? # TBD
                pass
            # end of if (0 < local_x)
            if (LOOKAHEAD_TIME_THRESHOLD <= lookahead_time) or (LOOKAHEAD_WPS <= waypoints_count):
                rospy.loginfo('Lookahead threshold reached: waypoints_count: %d; lookahead_time: %d'
                              % (waypoints_count, lookahead_time))
                break
            # end of if (LOOKAHEAD_WPS <= waypoints_count)
        # end of for waypoint in self.base_waypoints.waypoints
    
        # sort the waypoints by local_x increasing
        sorted_waypoints = sorted(waypoints_ahead, key=lambda x: x[1])  # sort by local_x
    
        # determine the speed at each waypoint
        final_waypoints = []
        for waypoint, local_x, local_y in sorted_waypoints:
            waypoint.twist.twist.linear.x = NORMAL_SPEED # meter/s, temporary hack for now
            final_waypoints.append(waypoint)
        # end of for waypoint, local_x, local_y
    
        # publish to /final_waypoints, need to package final_waypoints into Lane message
        publish_Lane(self.final_waypoints_pub, final_waypoints)

    def waypoints_cb(self, waypoints):
            # DONE: Implement
            self.base_waypoints = waypoints

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
