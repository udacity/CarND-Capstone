#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from styx_msgs.msg import Lane, TrafficLightArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, Int32, Header
from copy import deepcopy
from dynamic_reconfigure.server import Server
from visualization.cfg import DynReconfConfig

'''
This node publishes a visualization_marker_array topic for RViz to do 3D
visualization of the car driving around the waypoints and shows the ground
truth traffic light colors to compare with the detected status.
'''


class VisNode(object):

    def __init__(self):
        rospy.init_node('vis_node')

        self.vis_enabled = False  # default until dyn_reconf takes over
        self.vis_rate = 5  # default until dyn_reconf takes over

        self.subs = {}
        self.pubs = {}
        self.current_pose = None
        self.base_waypoints = []
        self.basewp_poses = []
        self.final_waypoints = []
        self.traffic_lights = []
        self.traffic_waypoint = -1

        self.pubs['/visualization_marker_array'] = rospy.Publisher(
                    '/visualization_marker_array', MarkerArray, queue_size=1)

        self.pubs['/visualization_basewp_path'] = rospy.Publisher(
                    '/visualization_basewp_path', Path, queue_size=1)

        self.subs['/current_pose'] = rospy.Subscriber(
                    '/current_pose', PoseStamped, self.handle_current_pose_msg)

        self.subs['/base_waypoints'] = rospy.Subscriber(
                    '/base_waypoints', Lane, self.handle_base_waypoints_msg)

        self.subs['/final_waypoints'] = rospy.Subscriber(
                    '/final_waypoints', Lane, self.handle_final_waypoints_msg)

        self.subs['/vehicle/traffic_lights'] = rospy.Subscriber(
                    '/vehicle/traffic_lights', TrafficLightArray,
                    self.handle_traffic_lights_msg)

        self.subs['/traffic_waypoint'] = rospy.Subscriber(
                    '/traffic_waypoint', Int32,
                    self.handle_traffic_waypoint_msg)

        self.dyn_reconf_srv = Server(DynReconfConfig, self.dyn_vars_cb)

        self.loop()

    # Callback to handle ground truth traffic lights message
    def handle_current_pose_msg(self, current_pose_msg):
        self.current_pose = current_pose_msg.pose

    # Callback to handle base waypoints message
    def handle_base_waypoints_msg(self, base_wp_msg):
        self.base_waypoints = []  # clear again just in case
        self.basewp_poses = []  # clear again just in case
        for wp_idx in range(len(base_wp_msg.waypoints)):
            self.base_waypoints.append(base_wp_msg.waypoints[wp_idx])
            if wp_idx % 100 == 0:  # thin out points for path msg
                self.basewp_poses.append(base_wp_msg.waypoints[wp_idx].pose)
        self.subs['/base_waypoints'].unregister()

    # Callback to handle final waypoints message
    def handle_final_waypoints_msg(self, final_wp_msg):
        self.final_waypoints = []
        for waypoint in final_wp_msg.waypoints:
            self.final_waypoints.append(waypoint)

    # Callback to handle ground truth traffic lights message
    def handle_traffic_lights_msg(self, traffic_lights_msg):
        self.traffic_lights = []
        for traffic_light in traffic_lights_msg.lights:
            self.traffic_lights.append(traffic_light)

    # Callback to handle detected traffic light waypoint message
    def handle_traffic_waypoint_msg(self, traffic_waypoint_msg):
        self.traffic_waypoint = traffic_waypoint_msg.data

    # Callback to adjust dynamic variables
    def dyn_vars_cb(self, config, level):

        if self.vis_rate != config['dyn_vis_rate']:
            self.vis_rate = config['dyn_vis_rate']  # store new rate
            self.rate = rospy.Rate(self.vis_rate)  # apply new rate

        self.vis_enabled = config['dyn_vis_enabled']  # store new flag

        return config

    # Helper function to set traffic light RGBA color from status #
    def set_traffic_light_color(self, tl_status, alpha):
        tl_color = ColorRGBA()
        tl_color.a = alpha
        if tl_status == 0:
            # Red light
            tl_color.r = 1.0
            tl_color.g = 0.0
            tl_color.b = 0.0
        elif tl_status == 1:
            # Yellow light
            tl_color.r = 1.0
            tl_color.g = 1.0
            tl_color.b = 0.0
        elif tl_status == 2:
            # Green light
            tl_color.r = 0.0
            tl_color.g = 1.0
            tl_color.b = 0.0
        else:
            # White light (unknown status)
            tl_color.r = 1.0
            tl_color.g = 1.0
            tl_color.b = 1.0
        return tl_color

    def loop(self):
        self.rate = rospy.Rate(self.vis_rate)

        while not rospy.is_shutdown():
            if self.vis_enabled:
                marker_array = MarkerArray()

                # Car Pose Arrow
                if self.current_pose is not None:
                    car_marker = Marker()
                    car_marker.id = 0
                    car_marker.header.frame_id = "/world"
                    car_marker.header.stamp = rospy.Time()
                    car_marker.type = Marker.ARROW
                    car_marker.action = Marker.ADD
                    car_marker.scale.x = 40.0
                    car_marker.scale.y = 8.0
                    car_marker.scale.z = 8.0
                    car_marker.color.a = 1.0
                    car_marker.color.r = 1.0
                    car_marker.color.g = 0.0
                    car_marker.color.b = 1.0
                    car_marker.pose = self.current_pose
                    marker_array.markers.append(car_marker)

                # Base Waypoint Line (switched to using built-in path instead)
                '''
                base_wp_line = Marker()
                base_wp_line.id = 1
                base_wp_line.header.frame_id = "/world"
                base_wp_line.header.stamp = rospy.Time()
                base_wp_line.type = Marker.LINE_STRIP
                base_wp_line.action = Marker.ADD
                base_wp_line.scale.x = 2.0
                base_wp_line.scale.y = 2.0
                base_wp_line.scale.z = 2.0
                base_wp_line.color.a = 1.0
                base_wp_line.color.r = 1.0
                base_wp_line.color.g = 1.0
                base_wp_line.color.b = 1.0
                for wp_idx in range(len(self.base_waypoints)):
                    if wp_idx % 30 == 0:
                        base_wp_line.points.append(
                                self.base_waypoints[wp_idx].pose.pose.position)
                marker_array.markers.append(base_wp_line)
                '''

                # Final Waypoint Line
                final_wp_line = Marker()
                final_wp_line.id = 2
                final_wp_line.header.frame_id = "/world"
                final_wp_line.header.stamp = rospy.Time()
                final_wp_line.type = Marker.LINE_STRIP
                final_wp_line.action = Marker.ADD
                final_wp_line.scale.x = 4.0
                final_wp_line.scale.y = 4.0
                final_wp_line.scale.z = 4.0
                final_wp_line.color.a = 1.0
                final_wp_line.color.r = 0.0
                final_wp_line.color.g = 1.0
                final_wp_line.color.b = 0.0
                # Make local copy for looping to prevent interrupt by callback
                final_waypoints = deepcopy(self.final_waypoints)
                for wp_idx in range(len(final_waypoints)):
                    if wp_idx % 10 == 0:  # thin out points
                        final_wp_line.points.append(
                                    final_waypoints[wp_idx].pose.pose.position)
                marker_array.markers.append(final_wp_line)

                # Traffic Waypoint Spheres
                tl_marker = Marker()
                tl_marker.id = 3
                tl_marker.header.frame_id = "/world"
                tl_marker.header.stamp = rospy.Time()
                tl_marker.type = Marker.SPHERE_LIST
                tl_marker.action = Marker.ADD
                tl_marker.scale.x = 30.0
                tl_marker.scale.y = 30.0
                tl_marker.scale.z = 30.0
                # Make local copy for looping to prevent interrupt by callback
                traffic_lights = deepcopy(self.traffic_lights)
                for tl_idx in range(len(traffic_lights)):
                    tl_marker.points.append(
                                traffic_lights[tl_idx].pose.pose.position)
                    tl_color = self.set_traffic_light_color(
                                        traffic_lights[tl_idx].state, 0.6)
                    tl_marker.colors.append(tl_color)
                marker_array.markers.append(tl_marker)

                # Detected Traffic Waypoint Red Cube
                if (len(self.base_waypoints) > 0
                        and self.traffic_waypoint >= 0
                        and self.traffic_waypoint < len(self.base_waypoints)):
                    det_tl_marker = Marker()
                    det_tl_marker.id = 4
                    det_tl_marker.header.frame_id = "/world"
                    det_tl_marker.header.stamp = rospy.Time()
                    det_tl_marker.type = Marker.CUBE
                    det_tl_marker.action = Marker.ADD
                    det_tl_marker.scale.x = 30.0
                    det_tl_marker.scale.y = 30.0
                    det_tl_marker.scale.z = 30.0
                    det_tl_marker.color.a = 1.0
                    det_tl_marker.color.r = 1.0
                    det_tl_marker.color.g = 0.0
                    det_tl_marker.color.b = 0.0
                    tl_idx = self.traffic_waypoint
                    # Make local copy to modify z position
                    tl_pose = deepcopy(self.base_waypoints[tl_idx].pose.pose)
                    tl_pose.position.z += 50  # offset the marker above waypnts
                    det_tl_marker.pose = tl_pose
                    marker_array.markers.append(det_tl_marker)

                # Publish final array of markers for RViz
                self.pubs['/visualization_marker_array'].publish(marker_array)

                # Publish base waypoint path for RViz
                basewp_header = Header()
                basewp_header.frame_id = "/world"
                basewp_header.stamp = rospy.Time()
                basewp_path = Path()
                basewp_path.header = basewp_header
                basewp_path.poses = self.basewp_poses
                self.pubs['/visualization_basewp_path'].publish(basewp_path)

            self.rate.sleep()


if __name__ == '__main__':
    VisNode()
