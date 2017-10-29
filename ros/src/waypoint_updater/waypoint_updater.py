#!/usr/bin/env python

import copy
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Float32

import math
import tf

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

# Local Constants
# ------------------------------------------------------------------------------

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number

SAMPLE_RATE = 20

# Maximum distance to the stop line when to start breaking.
MAX_DISTANCE_TO_STOP = 50.

# Local Helper-Functions
# ------------------------------------------------------------------------------

distance3d = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
distance2d = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

def get_pose_distance(pose1, pose2):
    ''' Computes the distance between to poses.
    Args:
        pose1, pose2: Poses to compute the distance between.
    Returns:
        Distance [m]
    '''
    return distance2d(pose1.position, pose2.position)

def waypoint_distance(waypoints, i1, i2):
    ''' Computes the distance between two waypoints in a list along
        the piecewise linear arc connecting all waypoints between the two.
    Args:
        waypoints: List of waypoints.
        i1, i2: Indices of two waypoints in the list.
    Returns:
        Distance [m]
    '''
    dist = 0
    for i in range(i1, i2):
        dist += distance3d(
            waypoints[i].pose.pose.position,
            waypoints[i+1].pose.pose.position)
    return dist

def get_waypoint_velocity(waypoint):
    ''' Provides the longitudinal velocity at a waypoint.
    Args:
        waypoint: Waypoint.
    Returns:
        Velocity [m/s]
    '''
    return waypoint.twist.twist.linear.x

def set_waypoint_velocity(waypoint, velocity):
    ''' Provides the longitudinal velocity at a waypoint.
    Args:
        waypoint: Waypoint.
        velocity: Velocity [m/s]
    '''
    waypoint.twist.twist.linear.x = velocity

def transform_coordinates(x0, y0, yaw0, xp, yp):
    ''' Computes the coordinates of a point in a new coordinate system.
    Args:
        x0, y0: Absolute coordinates of the origin of a new coordinate system.
        yaw:    Angle at which the new coordinate system is tilted.
        xp, yp: Absolute coordinates of a point.
    Returns:
        (x, y) coordinates of the point in the new coordinate system.
    '''
    # Compute coordinates of the point relative to the new origin.
    xr = xp - x0
    yr = yp - y0
    # Apply a 2D-transform of the point compensating for the tilt. Since the new
    # coordinate system is tilted by angle yaw0 in the absolute coordinate
    # system, rotation of the point must be done by angle -yaw0.
    cos_yaw = math.cos(-yaw0)
    sin_yaw = math.sin(-yaw0)
    return xr * cos_yaw - yr * sin_yaw, xr * sin_yaw + yr * cos_yaw

# WaypointUpdater
# ------------------------------------------------------------------------------

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Waypoint, self.obstacle_cb) # Implement later
        self.all_waypoints = []
        self.waypoints_ahead = []

        self.last_pose = None
        self.next_waypoint_ahead = None
        self.cte = 0.0
        self.next_stop_waypoint = None

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        self.next_waypoint_ahead_pub = rospy.Publisher(
            'next_waypoint_ahead', Int32, queue_size=1)

        self.cte_pub = rospy.Publisher('cte', Float32, queue_size=1)

        #rospy.spin()
        rate = rospy.Rate(SAMPLE_RATE)
        while not rospy.is_shutdown():
            self.publish_wp()
            rate.sleep()

    def publish_wp(self):
        if len(self.all_waypoints) > 0 and self.last_pose: # base waypoints published
            next_waypoint_ahead = self.get_next_waypoint_ahead()
            #if next_waypoint_ahead != self.next_waypoint_ahead:
            waypoints_ahead = self.get_waypoints_ahead(next_waypoint_ahead)
            waypoints_ahead = self.adjust_target_velocities(waypoints_ahead)
            self.publish(next_waypoint_ahead, waypoints_ahead)

    def pose_cb(self, msg):
        self.last_pose = msg.pose

    def waypoints_cb(self, waypoints):
        # reassign all waypoints of the track 
        # note: this should be a circular track
        self.all_waypoints = list(waypoints.waypoints)

    def traffic_cb(self, msg):
        # - msg is an index to all_waypoints[] indicating the position
        #   of the stop line of the next red light
        # - in case no red light is in sight, the value is -1
        self.next_stop_waypoint = msg.data
        if self.next_stop_waypoint != -1:
            rospy.loginfo("Stop waypoint is %s" % self.next_stop_waypoint)
        else:
            rospy.logdebug("No stop in sight")

    def obstacle_cb(self, msg):
        # TODO Callback for /obstacle_waypoint message. We will implement it later
        pass

    def adjust_target_velocities(self, waypoints_ahead):
        ''' Adjusts target velocity of waypoint in case of stopping.
        Args:
            waypoints_ahead: Waypoints ahead.
        Returns:
            Adjusted waypoints ahead.
        '''
        if self.next_stop_waypoint == None or self.next_stop_waypoint == -1:
            return waypoints_ahead
        distance_to_stop = waypoint_distance(self.all_waypoints,
                                             self.next_waypoint_ahead,
                                             self.next_stop_waypoint)
        rospy.logdebug("Next waypoint %d, next stop %d, distance to stop %f",
                       self.next_waypoint_ahead, self.next_stop_waypoint,
                       distance_to_stop)
        if distance_to_stop > MAX_DISTANCE_TO_STOP:
            return waypoints_ahead

        adjusted_waypoints_ahead = []
        if distance_to_stop < 0:
            # Stop at red light even if crossed the stop line.
            for i in range(len(waypoints_ahead)):
                adjusted_waypoint = copy.deepcopy(waypoints_ahead[i])
                set_waypoint_velocity(adjusted_waypoint, 0)
                adjusted_waypoints_ahead.append(adjusted_waypoint)
        else:
            # Linear deacceleration to the stop line.
            max_velocity = get_waypoint_velocity(waypoints_ahead[0]) \
                           * distance_to_stop / MAX_DISTANCE_TO_STOP
            waypoints_to_stop = self.next_stop_waypoint \
                - self.next_waypoint_ahead
            dv = max_velocity if waypoints_to_stop == 0 \
                else max_velocity / waypoints_to_stop
            for i in range(len(waypoints_ahead)):
                target_velocity = 0 if i >= waypoints_to_stop \
                    else (waypoints_to_stop - i) * dv
                adjusted_waypoint = copy.deepcopy(waypoints_ahead[i])
                set_waypoint_velocity(adjusted_waypoint, target_velocity)
                rospy.logdebug("Setting velocity of waypoint %d to %f",
                               i + self.next_waypoint_ahead, target_velocity)
                adjusted_waypoints_ahead.append(adjusted_waypoint)
        return adjusted_waypoints_ahead

    def yaw_from_quaternion(self, q):
        euler = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        return euler[2]

    def car_has_passed_waypoint(self, waypoint):
        # waypoint is the origo, and waypoint yaw x-axis direction
        x, y = transform_coordinates(
            waypoint.pose.pose.position.x,
            waypoint.pose.pose.position.y,
            self.yaw_from_quaternion(waypoint.pose.pose.orientation),
            self.last_pose.position.x,
            self.last_pose.position.y)
        self.cte = y
        return (x > 0.0) # car is ahead of the waypoint
       
    def add_waypoint_index(self, i, n):
        return (i + n) % len(self.all_waypoints)

    def find_nearest_waypoint(self):
        ''' Finds the waypoint nearest to the car.
        Returns:
          Index of the nearest waypoint.
        '''
        nwp = None
        for i in range(len(self.all_waypoints)):
            distance = get_pose_distance(self.last_pose,
                                         self.all_waypoints[i].pose.pose)
            if nwp == None or distance < closest_distance:
                nwp = i
                closest_distance = distance
        return nwp

    def find_first_waypoint_ahead(self):
        ''' Finst the nearest waypoint ahead of the car.
        Returns:
          Index of the nearest waypoint ahead.
        '''
        fwpa = self.find_nearest_waypoint()
        if self.car_has_passed_waypoint(self.all_waypoints[fwpa]):
            fwpa = self.add_waypoint_index(fwpa, 1)
        return fwpa

    def find_next_waypoint_ahead(self):
        nwpa1 = self.add_waypoint_index(self.next_waypoint_ahead, 1)
        nwpa2 = self.add_waypoint_index(nwpa1, 1)
        car = self.last_pose
        dist1 = get_pose_distance(car, self.all_waypoints[nwpa1].pose.pose)
        dist2 = get_pose_distance(car, self.all_waypoints[nwpa2].pose.pose)

        # go forward in the waypoint list in order to find local minimum distance
        while dist2 < dist1:
            dist1 = dist2
            nwpa1 = nwpa2
            nwpa2 = self.add_waypoint_index(nwpa2, 1)
            dist2 = get_pose_distance(car, self.all_waypoints[nwpa2].pose.pose)

        # take next if we have already passed the closest one
        if self.car_has_passed_waypoint(self.all_waypoints[nwpa1]):
            nwpa1 = nwpa2

        return nwpa1

    def get_next_waypoint_ahead(self):
        cwpa = self.next_waypoint_ahead # current waypoint ahead
        next_waypoint_ahead = cwpa
        if cwpa == None:
            next_waypoint_ahead = self.find_first_waypoint_ahead()
        elif self.car_has_passed_waypoint(self.all_waypoints[cwpa]):
            next_waypoint_ahead = self.find_next_waypoint_ahead()

        # debugging
        if next_waypoint_ahead != cwpa:
            dist = get_pose_distance(
                self.last_pose,
                self.all_waypoints[next_waypoint_ahead].pose.pose)
            dist2 = get_pose_distance(
                self.last_pose,
                self.all_waypoints[next_waypoint_ahead-1].pose.pose)
            dist3 = get_pose_distance(
                self.all_waypoints[next_waypoint_ahead-1].pose.pose,
                self.all_waypoints[next_waypoint_ahead].pose.pose)
            rospy.loginfo(
                "waypoint_updater: next waypoint %s, car position (%s,%s,%s)",
                next_waypoint_ahead,
                self.last_pose.position.x,
                self.last_pose.position.y,
                self.yaw_from_quaternion(self.last_pose.orientation))

        return next_waypoint_ahead

    def get_waypoints_ahead(self, i):
        waypoints = []

        if self.next_waypoint_ahead == i:
            waypoints = self.waypoints_ahead
        else: 
            self.next_waypoint_ahead = i

            first_waypoint = self.next_waypoint_ahead
            last_waypoint  = self.add_waypoint_index(first_waypoint, LOOKAHEAD_WPS)

            if first_waypoint < last_waypoint:
                waypoints = self.all_waypoints[first_waypoint:last_waypoint+1]
            else:
                waypoints = self.all_waypoints[first_waypoint:]
                waypoints.extend(self.all_waypoints[:last_waypoint+1])

            self.waypoints_ahead = waypoints

        return waypoints

    def publish(self, next_waypoint_ahead, waypoints_ahead):
        lane = Lane()

        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)

        lane.waypoints = waypoints_ahead

        self.next_waypoint_ahead_pub.publish(next_waypoint_ahead)
        self.cte_pub.publish(self.cte)

        self.final_waypoints_pub.publish(lane)



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
