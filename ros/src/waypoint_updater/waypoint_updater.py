#!/usr/bin/env python

import tf
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
from std_msgs.msg import Int32

import math
import yaml

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
DECELERATION_DISTANCE = 50.  # 50 meters

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        config_string = rospy.get_param("/traffic_light_config")
        self.traffic_light_config = yaml.load(config_string)

        self.stop_lines = []
        self.deceleration_points = []
        self.base_waypoints = []
        self.reference_velocity = []
        self.pose = None
        self.red_light_stop_point = -1
        self.loop()

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)


    # Main loop.  if pose and waypoints are both receved, compute final waypoints every 100ms.
    # Gradually slow down and stop for any red lights.

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.pose != None and len(self.base_waypoints) != 0 and len(self.stop_lines) != 0:
                # Find car position and direction
                car_x = self.pose.position.x
                car_y = self.pose.position.y
                quaternion = (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
                car_yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
                #print("Car is at %f, %f and facing %f" % (car_x, car_y, car_yaw))
                
                # Find nearest waypoint
                min_distance = float("inf")
                min_distance_waypoint = 0
                for i in range(0, len(self.base_waypoints)):
                    waypoint = self.base_waypoints[i]
                    waypoint_x = waypoint.pose.pose.position.x
                    waypoint_y = waypoint.pose.pose.position.y
                    distance = math.sqrt((car_x-waypoint_x)**2 + (car_y-waypoint_y)**2)
                    if distance < min_distance:
                        min_distance = distance
                        min_distance_waypoint = i

                # Nearest waypoint may be behind us!  Check this by transforming waypoint to
                # car's coordinate system.
                waypoint_x = self.base_waypoints[min_distance_waypoint].pose.pose.position.x
                waypoint_y = self.base_waypoints[min_distance_waypoint].pose.pose.position.y
                #print("Closest waypoint is %d, at %f, %f (distance %f)" % (min_distance_waypoint, waypoint_x, waypoint_y, min_distance))
                transformed = self.transform(waypoint_x, waypoint_y, -car_x, -car_y, -car_yaw)
                #print("Transformed: %f, %f" % (transformed[0], transformed[1]))
                
                # If x coordinate of nearest waypoint is negative, that means it's behind us.
                # Use next waypoint instead.
                if transformed[0] < 0:
                    #print("Closest waypoint is behind us, using the next one")
                    min_distance_waypoint = min_distance_waypoint+1

                last_waypoint = min(len(self.base_waypoints) - 1, min_distance_waypoint + LOOKAHEAD_WPS)

                # Set each waypoint back to its initial target velocity
                for waypoint in range(min_distance_waypoint, last_waypoint):
                    self.base_waypoints[waypoint].twist.twist.linear.x = self.reference_velocity[waypoint]

                deceleration_point = -1

                # Slow down gradually for any red lights ahead
                if self.red_light_stop_point != -1:
                    light = self.red_light_stop_point
                    #print("will stop at waypoint %d" % light)
                    full_speed = self.reference_velocity[light]
                    #print("full speed is %f" % full_speed)
                    deceleration_point = self.deceleration_points[self.stop_lines.index(light)]
                    #print("decel point is %d" % deceleration_point)
                    for i in range(deceleration_point, light):
                        distance = self.distance(self.base_waypoints, i, light)
                        speed = full_speed * distance / DECELERATION_DISTANCE
                        self.base_waypoints[i].twist.twist.linear.x = max(speed, .5)
                        #print("speed at waypoint %d is %f" % (i, speed))
                    self.base_waypoints[light].twist.twist.linear.x = 0
                    #print("speed at waypoint %d is %f" % (light, 0))

                # Publish at most LOOKAHEAD_WPS waypoints
                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time(0)
                lane.waypoints = self.base_waypoints[min_distance_waypoint:last_waypoint]
                self.final_waypoints_pub.publish(lane)

                rospy.loginfo('waypoint_updater: %d waypoints (%d-%d), stop: %d slow: %d target speed: %f' %
                    (len(lane.waypoints), min_distance_waypoint, last_waypoint, self.red_light_stop_point,
                    deceleration_point, self.base_waypoints[min(len(self.base_waypoints) - 1, min_distance_waypoint)].twist.twist.linear.x))
        
            rate.sleep()

    def lights_cb(self, msg):
        self.lights = msg.lights

    def pose_cb(self, msg):
        self.pose = msg.pose

    # Receive and process waypoints.  Also find nearest waypoint to each stop line,
    # and point to begin decelerating for each stop line
    def waypoints_cb(self, waypoints):
        rospy.loginfo('waypoint_updater: received %d waypoints' % (len(waypoints.waypoints)))
        self.base_waypoints = waypoints.waypoints
        # keep a copy of original speed limit for each waypoint
        for i in range(0, len(self.base_waypoints)):
            self.reference_velocity.append(self.base_waypoints[i].twist.twist.linear.x)
        # find stop line waypoints for each light
        stop_line_positions = self.traffic_light_config['stop_line_positions']
        #print("there are %d lights" % len(stop_line_positions))
        rospy.loginfo('waypoint_updater: there are %d lights!' % (len(stop_line_positions)))
        for position in stop_line_positions:
            waypoint = self.nearest_waypoint(position[0], position[1])
            self.stop_lines.append(waypoint)
            #print("stop line at %d" % waypoint)
        for waypoint in self.stop_lines:
            for deceleration_point in range(waypoint, 0, -1):
                distance = self.distance(self.base_waypoints, deceleration_point, waypoint)
                #print("distance from %d to %d: %f" % (waypoint, deceleration_point, distance))
                if distance > 50:
                    self.deceleration_points.append(deceleration_point)
                    break


    def traffic_cb(self, msg):
        if self.red_light_stop_point != msg.data:
            rospy.loginfo('waypoint_updater: red light at waypoint %d' % msg.data)
        self.red_light_stop_point = msg.data
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

    def nearest_waypoint(self, x, y):
        min_distance = float("inf")
        min_distance_waypoint = 0
        for i in range(0, len(self.base_waypoints)):
            waypoint = self.base_waypoints[i]
            waypoint_x = waypoint.pose.pose.position.x
            waypoint_y = waypoint.pose.pose.position.y
            distance = math.sqrt((x-waypoint_x)**2 + (y-waypoint_y)**2)
            if distance < min_distance:
                min_distance = distance
                min_distance_waypoint = i
        return min_distance_waypoint

    # transform coordinate systems
    def transform(self, x, y, x_offset, y_offset, rotation):
        x = x + x_offset
        y = y + y_offset
        new_x = x * math.cos(rotation) - y * math.sin(rotation)
        new_y = x * math.sin(rotation) + y * math.cos(rotation)
        return [new_x, new_y]


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
