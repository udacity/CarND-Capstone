#!/usr/bin/env python
"""
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
"""

from geometry_msgs.msg import PoseStamped
import math
import rospy
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray


def get_waypoint_velocity(waypoint):
    """Get the velocity from a given waypoint."""
    return waypoint.twist.twist.linear.x


def set_waypoint_velocity(waypoints, index, velocity):
    """Set the velocity of the waypoint at the given index."""
    waypoints[index].twist.twist.linear.x = velocity


def distance(waypoints, index_1, index_2):
    """Calculate the distance between two waypoints using a piece-wise linear function."""
    dist = 0
    dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
    for i in range(index_1, index_2+1):
        dist += dl(waypoints[index_1].pose.pose.position, waypoints[i].pose.pose.position)
        index_1 = i
    return dist


class WaypointUpdater(object):
    """A waypoint updater node implemented as a Python class."""
    RUNNING = 0
    STOPPING = 1
    STOPPED = 2
    ACCELERATING = 3

    def __init__(self):
        """
        Constructor.

        - Subscribes to current_pose and base_waypoints
        - Gets the lookahead parameter ~lookahead_wps
        - Advertises to final_waypoints
        - Sets up class variables
        """
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.gt_traffic_cb)
        self.lookahead_wps = rospy.get_param('~lookahead_wps', 200)
        self.max_decel = abs(rospy.get_param('~max_deceleration', 1.0))

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.next_traffic = TrafficLight()
        self.valid_next_traffic = False
        self.dist_threshold = 2.
        self.lane = Lane()
        self.last_lane = Lane()
        self.nb_stopping_wp = 30

        self.traffic_lights = TrafficLightArray()

        self.state = WaypointUpdater.ACCELERATING

        self.traffic_light_wp_index = -1

        self.commanded_velocity = 0

        self.cruise_velocity = 4.47 

        self.currpose = None
        self.curr_waypoints = None

    def pose_cb(self, msg):
        """Callback for the curr_pose just stores it for later use."""
        self.currpose = msg.pose.position

    def waypoints_cb(self, lanemsg):
        """
        Callback for base_waypoints finds the waypoint.

        Finds the waypoint closes to the current pose, then publishes the next lookahead_wps
        waypoints after that one to final_waypoints.
        """
        if self.currpose is None:
            return

        self.curr_waypoints = lanemsg.waypoints
        self.lane = Lane()
        self.lane.header.frame_id = '/world'
        self.lane.header.stamp = rospy.Time(0)
        mindist = 1000000
        start_idx = 0

        for i in range(len(self.curr_waypoints)):
            a = self.curr_waypoints[i].pose.pose.position
            b = self.currpose
            dist = math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
            # if dist < mindist and (a.x > b.x):
            if dist < mindist:
                start_idx = i
                mindist = dist
            if start_idx == (len(self.curr_waypoints) - 1):
                start_idx = 0

        idx = 0
        reset = 0
        # Collecting the waypoints ahead of the car.
        # Wrap around when we reach the end.
        for i in range(self.lookahead_wps):
            idx = (start_idx + i) % len(self.curr_waypoints)
            self.lane.waypoints.append(self.curr_waypoints[idx])

        index = -1
        # Check if there is a traffic light on the lookahead path
        self.valid_next_traffic = False
        for i in range(len(self.traffic_lights.lights)):
            index = self.closest_index(self.traffic_lights.lights[i])
            if(index >= self.nb_stopping_wp-5):
                self.next_traffic = self.traffic_lights.lights[i]
                self.valid_next_traffic = True
                break
        self.traffic_light_wp_index = index

        self.step()
       

        self.last_lane = self.lane
        self.final_waypoints_pub.publish(self.lane)

    def step(self):
        # print(self.state)
        # print("x: ", self.currpose.x)

        if(len(self.last_lane.waypoints)>0):
            self.lane = self.keep_last(self.lane, self.last_lane)
            self.commanded_velocity = get_waypoint_velocity(self.lane.waypoints[0])
        else:
            self.lane = self.keep_speed(self.lane, self.cruise_velocity)
            self.commanded_velocity = self.cruise_velocity
        
        if(self.state == WaypointUpdater.RUNNING):
            # print("RUNNING")
            if(self.valid_next_traffic and 
                (self.next_traffic.state is TrafficLight.RED or 
                    self.next_traffic.state is TrafficLight.YELLOW and self.traffic_light_wp_index>1.5*self.nb_stopping_wp)):
                self.state = self.STOPPING
                self.lane = self.breaking(self.lane, self.traffic_light_wp_index-self.nb_stopping_wp, self.cruise_velocity) # TRANSITION TO STOPPING
            
        elif(self.state == WaypointUpdater.STOPPING):
            # print("STOPPING")
            if(self.valid_next_traffic and self.next_traffic.state is TrafficLight.GREEN):
                self.state = self.ACCELERATING
                self.lane = self.keep_speed(self.lane, self.cruise_velocity)
            elif(self.commanded_velocity<0.5):
                self.state = self.STOPPED
                self.lane = self.keep_speed(self.lane, 0)                   # TRANSITION TO STOPPED
        elif(self.state == WaypointUpdater.STOPPED):
            # print("STOPPED")
            if(self.next_traffic.state == TrafficLight.GREEN or self.valid_next_traffic is False):
                self.state = WaypointUpdater.ACCELERATING
                self.lane = self.keep_speed(self.lane, self.cruise_velocity) # TRANSITION TO ACCELERATING
        elif(self.state == WaypointUpdater.ACCELERATING):
            # print("ACCELERATING")
            if(self.commanded_velocity>self.cruise_velocity-0.5):
                self.state = self.RUNNING
                self.lane = self.keep_speed(self.lane, self.cruise_velocity) # TRANSITION TO RUNNING


    def keep_last(self, base_lane, last_lane):

        min_dist, curr_index = self.find_index(base_lane.waypoints[0].pose.pose.position, last_lane.waypoints)

        # velocity = 
        new_waypoint_list = last_lane.waypoints[curr_index:]

        last_vel = get_waypoint_velocity(last_lane.waypoints[-1])
        for i in range(len(new_waypoint_list), len(base_lane.waypoints)):
            set_waypoint_velocity(base_lane.waypoints, i, last_vel)
            new_waypoint_list.append(base_lane.waypoints[i])

        base_lane.waypoints = new_waypoint_list

        for i in range(len(base_lane.waypoints)):
            velocity = get_waypoint_velocity(base_lane.waypoints[i])
            # print("index: ", i, "velocity: ",velocity, "x: ", base_lane.waypoints[i].pose.pose.position.x)

        # for i in range(len(last_lane.waypoints) - curr_index, len(base_lane.waypoints)):
        #     set_waypoint_velocity(base_lane.waypoints, i, velocity)

        return base_lane

    def keep_speed(self, base_lane, velocity):

        for i in range(len(base_lane.waypoints)):
            set_waypoint_velocity(base_lane.waypoints, i, velocity)

        return base_lane

    def accelerate(self, base_lane, end_velocity, ramp_length):

        return base_lane

    def breaking(self, base_lane, stop_index, curr_vel):
        for i in range(stop_index+1):
            wp_distance = distance(base_lane.waypoints, i, stop_index) - 4.0 # This is a hack for stop lines
            wp_distance = max(0, wp_distance)
            velocity = min(curr_vel, math.sqrt(2 * wp_distance * self.max_decel))
            set_waypoint_velocity(base_lane.waypoints, i, velocity)  

        for i in range(stop_index+1, len(base_lane.waypoints)):
            set_waypoint_velocity(base_lane.waypoints, i, 0)

        return base_lane


    def find_index(self, position, waypoint_list):
        min_index = -1
        min_dist = 9999999.

        for i in range(len(waypoint_list)):
            wp_position = waypoint_list[i].pose.pose.position
            dist = math.sqrt((position.x - wp_position.x)**2 + (position.y - wp_position.y)**2)
            if(dist < min_dist):
                min_dist = dist
                min_index = i

        return [min_dist, min_index]



    def closest_index(self, traffic_light):
        """ Verifies if the traffic light is on the current lookahead path"""

        # print(len(self.lane.waypoints))
        t_position = traffic_light.pose.pose.position
        wps = self.lane.waypoints
        for i in range(len(wps)):
            wp_position = wps[i].pose.pose.position
            dist = math.sqrt((t_position.x - wp_position.x)**2 + (t_position.y - wp_position.y)**2)
            if(dist < self.dist_threshold):
                return i
            
        # print("traffic_light: ", traffic_light)
        # print("min_dist: ", min_dist)

        # if(min_dist < self.dist_threshold):
        #     return True

        return -1

    def gt_traffic_cb(self, msg):
        """Callback for the ground truth traffic_waypoint message."""
        self.traffic_lights = msg
        # self.valid_next_traffic = False
        # for i in range(len(msg.lights)):
        #     if(self.is_on_lookahead(msg.lights[i])):
        #         self.next_traffic = msg.lights[i]
        #         # print("next_traffic: ", self.next_traffic)
        #         self.valid_next_traffic = True
        #         break

        # if(self.valid_next_traffic is False):
        #     print("No Traffic Light in the lookahead path")            


    def traffic_cb(self, msg):
        """Callback for the traffic_waypoint message."""
        self.traffic_lights = TrafficLightArray()
        self.traffic_lights.append(msg)
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        """Callback for the obstacle_waypoint message."""
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass


if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_updater')
        node = WaypointUpdater()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
