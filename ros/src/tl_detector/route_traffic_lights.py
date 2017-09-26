#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane, Waypoint
import tf
import math

class RouteTrafficLight(object):
    def __init__(self, x, y):
        self.stop_line_position = Point(x,y,0) 
        self.pose = None
        self.state = TrafficLight.UNKNOWN
        self.stop_point = None

class RouteTrafficLights(object):

    '''
    Traffic lights along a route
    Traffic light position data container class
    '''

    def __init__(self):
        self.traffic_lights = []
        self.n_waypoints = 0
        self.first = 10e9
        self.last = 0
        self.pose_updated = False

    def __getitem__(self, key):
        return self.traffic_lights[key]

    def set_stop_points(self, waypoints):

        '''
        Assign a stop point for each traffic light
        A stop point is the closest waypoint before the stop line
        '''

        # FIXME Figure out something more elegant than brute force
        # FIXME Stop point should be the closest point before the stop line

        for tl in self.traffic_lights:
            dmin = 10e9
            i = 0
            for j in range(len(waypoints)):
                wp = waypoints[j].pose.pose.position
                sl = tl.stop_line_position
                d = math.sqrt((sl.x-wp.x)**2+(sl.y-wp.y)**2)
                if d < dmin:
                    i = j
                    dmin = d

            tl.stop_point = i

    def reset(self, waypoints, stop_line_positions):

        '''
        Reset the container with new route information 
        Args:
            waypoints: waypoints of the route
            stop_line_positions: stop line positions of traffic lights along
                the route
        '''

        self.traffic_lights = []
        for p in stop_line_positions:
            self.traffic_lights.append(RouteTrafficLight(p[0],p[1]))

        assert len(stop_line_positions) == len(self.traffic_lights)

        self.pose_updated = False
        self.n_waypoints = len(waypoints)
        self.set_stop_points(waypoints)

        for tl in self.traffic_lights:
            assert tl.stop_point != None
            if tl.stop_point < self.first:
                self.first = tl.stop_point
            if tl.stop_point > self.last:
                self.last = tl.stop_point

    def update_states(self, traffic_light_array):
        '''
            Update states of all traffic lights.
            This is for simulation when we get the state of 
            all traffic lights from /vehicle/traffic_lights topic
        '''
     
        # FIXME For now we assume that the order and size of traffic light
        # array and stop line positions are the same. 

        for tl, i in zip(traffic_light_array,range(len(traffic_light_array))):
            assert i < len(self.traffic_lights)
            if tl.state != TrafficLight.UNKNOWN:
                self.traffic_lights[i].state = tl.state
            if self.pose_updated == False:
                self.traffic_lights[i].pose = tl.pose.pose 
        self.pose_updated = True

    def get_next_en_route(self, car):
        '''
            Return stop point of next traffic light en route
            Arg: car = car's next waypoint ahead
            Return: key (index)
        '''

        next_p = None

        if car > self.last:
            next_p = self.first # NOTE assume circular route
        elif car <= self.first:
            next_p = self.first
        else:
            next_p = self.last
            for tl in self.traffic_lights:
                if tl.stop_point > car and tl.stop_point < next_p:
                    next_p = tl.stop_point

        assert next_p != None

        next_i = None
        for i in range(len(self.traffic_lights)):
            if self.traffic_lights[i].stop_point == next_p:
                next_i = i

        return next_p, next_i

# smoke test
if __name__ == '__main__':
    import csv
    import yaml
    from sys import exit

    fname = "../../../data/wp_yaw_const.csv"
    waypoints = []

    with open(fname) as wfile:
        reader = csv.DictReader(wfile,  ['x', 'y', 'z', 'yaw'])
        for wp in reader:
            p = Waypoint()
            p.pose.pose.position.x = float(wp['x'])
            p.pose.pose.position.y = float(wp['y'])
            p.pose.pose.position.z = float(wp['z'])
            q = tf.transformations.quaternion_from_euler(0., 0., float(wp['yaw']))
            p.pose.pose.orientation = Quaternion(*q)
            p.twist.twist.linear.x = float(40*0.27778)

            waypoints.append(p)

    with open('sim_traffic_light_config.yaml') as f:
        try:
            config = yaml.load(f)
        except yaml.YAMLError as exc:
            print exc
            exit()
     
    # module tests (depend on the config and waypoint files)
    lights = RouteTrafficLights()

    stop_line_positions = config['stop_line_positions']
    lights.reset(waypoints, stop_line_positions)
    print lights.get_next_en_route(5000)
    print lights.get_next_en_route(100)
    print lights.get_next_en_route(1000)
    print lights.get_next_en_route(10000)
