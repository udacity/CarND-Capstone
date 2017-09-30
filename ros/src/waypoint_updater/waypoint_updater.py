#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

'''

## Control Parameters
# Number of waypoints we will publish. You can change this number
LOOKAHEAD_WPS = 200 
# Car speed in simulator (or real env) is MPH, whereas ROS uses MPS
MPH_TO_MPS = 0.44704
# Max car speed
MAX_SPEED = 10 * MPH_TO_MPS #m/s 


class WaypointUpdater(object):
    """The purpose of the node is to publish a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights and obstacles.

        - subscribed topics:
            - `/base_waypoints`: repeated list of all points
            - `/obstacle_waypoints`: NOT THERE!
            - `/traffic_waypoint`
            - `/current_pose`
        - published topics:
            - `/final_waypoints`: only waypoints ahead up to `LOOKAHEAD_WPS`.
        - node files:
            - `waypoint_updater/waypoint_updater.py`
    """
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb) # 40-45 hz
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb) # 40 hz
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb) # 10 hz
        rospy.Subscriber('/obstacle_waypoints', PoseStamped, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # asychronizedly receive data from subscried topics
        ## all waypoints on the map
        self.base_waypoints_msg = None
        ## current car pose (position and orientation)
        self.car_pose = None
        ## traffic way point index
        self.redlight_wp_index = None

        ## rate to publish to final_waypoints
        self.publish_rate = 40 # doesnt make sense if >= 40, which is /current_pose rate


        # publish waypoints in a loop with explicit rate
        self.loop()

        rospy.spin()

    ############# Callback for subscription ##################################

    def pose_cb(self, msg):
        # update it everytime when received
        self.car_pose = msg.pose

    def waypoints_cb(self, waypoints):
        # 10902 waypoints for the simulation env
        self.base_waypoints_msg = waypoints
        # receive once
        self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        # index of next RED light in the base_waypionts list
        self.redlight_wp_index = msg.data
        if self.redlight_wp_index >= len(self.base_waypoints_msg.waypoints):
            rospy.logerr("Traffic light prediction error!")

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    ###################### Pulishing loop ###################################

    def loop(self):
        """Publish to /final_waypoints with waypoints ahead of car
        """
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            # publishing nothing when receiving nothing
            if (self.car_pose is None) or (self.base_waypoints_msg is None):
                continue

            ## decide target speed based on traffic light
            target_speed = MAX_SPEED
            if self.red_light_ahead():
                target_speed = 0

            ## setup lane for final_waypoints
            frame_id = self.base_waypoints_msg.header.frame_id
            lane_start = self.next_waypoint()
            waypoints = self.base_waypoints_msg.waypoints[lane_start:lane_start+LOOKAHEAD_WPS]
            for waypoint in waypoints:
                self.set_waypoint_velocity(waypoint, target_speed)

            # stop at the end of road
            if lane_start + LOOKAHEAD_WPS >= len(self.base_waypoints_msg.waypoints):
                
		for waypoint in waypoints[-10:]:
                    self.set_waypoint_velocity(waypoint, 0.)
            lane = self.make_lane_msg(frame_id, waypoints)

            ## publish final_waypoints
            self.final_waypoints_pub.publish(lane)

            rate.sleep()

    ######################### Helper functions ################################

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def get_waypoint_coordinates(self, waypoint):
        wp_x = waypoint.pose.pose.position.x
        wp_y = waypoint.pose.pose.position.y
        wp_z = waypoint.pose.pose.position.z
        return (wp_x, wp_y, wp_z)

    def get_car_coordinates(self, car_pose):
        car_x = car_pose.position.x
        car_y = car_pose.position.y
        car_z = car_pose.position.z
        return (car_x, car_y, car_z)

    def get_car_euler(self, car_pose):
        """Return roll, pitch, yaw from car's pose
        """
        return tf.transformations.euler_from_quaternion([
            car_pose.orientation.x,
            car_pose.orientation.y,
            car_pose.orientation.z,
            car_pose.orientation.w])

    def make_lane_msg(self, frame_id, waypoints):
        """This should be the job of lane constructor
        """
        lane = Lane()
        lane.header.frame_id = frame_id
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        return lane

    def distance(self, waypoint, car_pose):
        wp_x, wp_y, wp_z = self.get_waypoint_coordinates(waypoint)
        car_x, car_y, car_z = self.get_car_coordinates(car_pose)

        dx = wp_x - car_x
        dy = wp_y - car_y
        dz = wp_z - car_z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def inner_product(self, vec1, vec2):
        return sum([v1*v2 for v1, v2 in zip(vec1, vec2)])

    def ahead_of(self, waypoint, car_pose):
        """If a waypoint is ahead of the car based on its current pose.
        Logic: In the local coordinate system (car as origin), the angle
        between the waypoint vector and the car's current yaw vector should be
        less than 90, which also means their innter product should be positive.
        """
        wp_x, wp_y, wp_z = self.get_waypoint_coordinates(waypoint)
        car_x, car_y, car_z = self.get_car_coordinates(car_pose)
        _, _, car_yaw = self.get_car_euler(car_pose)

        wp_vector = (wp_x-car_x, wp_y-car_y)
        yaw_vector = (math.cos(car_yaw), math.sin(car_yaw))

        return self.inner_product(wp_vector, yaw_vector) > 0


    def red_light_ahead(self):
        
        if self.redlight_wp_index is None or self.base_waypoints_msg is None:
            return False
        elif self.redlight_wp_index >= len(self.base_waypoints_msg.waypoints): 
        # traffic light prediction error, stop immediately
            return True
        else:
            base_waypoints = self.base_waypoints_msg.waypoints
            light_wp = base_waypoints[self.redlight_wp_index]
            distance = self.distance(light_wp, self.car_pose)
            # stops in x distance 
            if self.ahead_of(light_wp, self.car_pose) and distance <= 35: 
                return True
            else:
                return False

    def next_waypoint(self):
        """Get the index of next waypoint ahead of the car, based on
        received current_pose of car and base_waypoints
        """
        waypoints = self.base_waypoints_msg.waypoints
        next_wp_index = 0
        next_wp_dist = float('inf')
        for i, wp in enumerate(waypoints):
            if not self.ahead_of(wp, self.car_pose):
                continue
            else:
                d = self.distance(wp, self.car_pose)
                if d < next_wp_dist:
                    next_wp_dist = d
                    next_wp_index = i
        return next_wp_index


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
