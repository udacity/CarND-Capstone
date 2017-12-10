#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightWaypoint, TrafficLightState
from std_msgs.msg import Int32
import numpy as np
import math, time
import scipy.linalg as la
from tf.transformations import euler_from_quaternion

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
        rospy.init_node('waypoint_updater')

        speed_limit = rospy.get_param('/waypoint_loader/velocity')
        self.speed_limit = speed_limit * 1000.0 / 3600.0
        rospy.logwarn("speed limit = {} kph ({} mps).".format(speed_limit, self.speed_limit))

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', TrafficLightWaypoint, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', , self.waypoints_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.velocity = 0.0
        self.traffic_waypoint = -1
        self.approach_velocity = None
        self.way_points = None
        self.way_points_np = None
        self.pose = None
        self.poses = 0

        rospy.spin()

    def velocity_cb(self, msg):
        # /current_velocity message (units: m/s)
        # The linear velocity is in the vehicle's frame of reference so only 'x' is populated
        self.velocity = msg.twist.linear.x

    def pose_cb(self, msg):
        self.pose =  msg.pose

        self.poses += 1

        if self.way_points and self.pose: #run once self.way_points has been populated

            #get index of top LOOKAHEAD_WPS points, vectorized function for fast computation
            final_waypoints_index = self.index_of_nearest_wpts(self.way_points_np, self.pose)
            #get the selected waypoints
            final_waypoints = np.array(self.way_points)[final_waypoints_index].tolist()

            stopping = False

            if self.traffic_waypoint == -1:
                self.approach_velocity = None
            else:
                # Drop velocity to zero by the time the traffic_waypoint is reached.
                # Not sure of requirement if it is suddenly 'RED' when at an impossible
                # breaking distance.
                nearest_waypoint = final_waypoints_index[0]
                distance = self.distance(self.way_points, nearest_waypoint, self.traffic_waypoint)
                if self.approach_velocity == None:
                    self.approach_velocity = self.velocity
                    rospy.logwarn("distance to stopline: {:.5}m, approaching velocity: {:.5}m/s.".format(distance, self.velocity))
                # Check if we can or should stop
                # TODO: Distance for Yellow Lights needs to be calculated
                if self.traffic_state == TrafficLightState.RED or (self.traffic_state == TrafficLightState.YELLOW and distance>50.0):
                    stopping = True
                    points = self.traffic_waypoint - nearest_waypoint
                    for i, waypoint in enumerate(range(nearest_waypoint, self.traffic_waypoint)):
                        self.set_waypoint_velocity(self.way_points, waypoint%self.way_points_count, self.velocity * (points-i-1)/points)
                    for waypoint in range(self.traffic_waypoint, self.traffic_waypoint + LOOKAHEAD_WPS - points):
                        self.set_waypoint_velocity(self.way_points, waypoint%self.way_points_count, 0.0)

            if not stopping:
                # Set velocity to maximum (dbw_node controller to handle correct acceleration)
                nearest_waypoint = final_waypoints_index[0]
                for i, waypoint in enumerate(range(nearest_waypoint, nearest_waypoint + LOOKAHEAD_WPS)):
                    self.set_waypoint_velocity(self.way_points, waypoint%self.way_points_count, self.speed_limit)

            pub = Lane()
            pub.header = msg.header
            pub.waypoints = final_waypoints
            self.final_waypoints_pub.publish(pub)

    def waypoints_cb(self, waypoints):
        #store waypoints' x,y,z in numpy array so we use vectorization in numpy
        self.way_points_np = np.array([[wp.pose.pose.position.x, wp.pose.pose.position.y, wp.pose.pose.position.z] for wp in waypoints.waypoints])
        #store waypoints
        self.way_points = waypoints.waypoints
        self.way_points_count = len(self.way_points)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint = int(msg.waypoint)
        self.traffic_state = msg.state.state

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def index_of_nearest_wpts(self, wpts_np, pose):
        #transform wpts to coordinate system centered on the car but retaining original orientation
        transformed_wpts = wpts_np - np.array([pose.position.x, pose.position.y, 0]) #-
        #roll, pitch, yaw, we only need yaw
        _, _, yaw = (euler_from_quaternion((self.pose.orientation.x,self.pose.orientation.y, \
        self.pose.orientation.z,self.pose.orientation.w)))

        #Convert Yaw to a direction vec
        direction_vec = np.array([np.cos(yaw), np.sin(yaw), 0])

        #project the waypoints on to the direction vector of the car, positive values will be ahead, negative behind
        #The Following is the same as np.dot, only fasterprojected_values = (np.dot(transformed_wpts,direction_vec.T))
        projected_values = la.blas.dgemm(1.0,transformed_wpts,direction_vec.T).reshape(-1)

        #calculate distance of each way point from car
        distances = np.sqrt((wpts_np[:,0] - pose.position.x)**2 + (wpts_np[:,1] - pose.position.y)**2 + (wpts_np[:,2] - pose.position.z)**2)

        #set all projected values to either 0 or 1
        projected_values[projected_values >= 0] = 1
        projected_values[projected_values < 0]  = 0
        #assign 0 to waypoints behind the car 0
        distances = np.multiply(projected_values , distances )
        #assign these waypoints a high distance value
        distances[distances == 0] = 1e6
        #return the top LOOKAHEAD_WPS coordinates
        return np.argsort(distances)[:LOOKAHEAD_WPS]

    def distance(self, waypoints, wp1, wp2):
        dist = 0.0
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
