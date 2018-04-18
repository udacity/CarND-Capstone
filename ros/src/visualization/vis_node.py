#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from styx_msgs.msg import Lane, TrafficLightArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, Int32, Header
from copy import deepcopy

'''
This node publishes a visualization_marker_array topic for RViz to do 3D
visualization of the car driving around the waypoints and shows the ground
truth traffic light colors to compare with the detected status.
'''


class VisNode(object):

    def __init__(self):
        rospy.init_node('vis_node')

        self.vis_enabled = rospy.get_param('~vis_enabled')
        self.vis_rate = rospy.get_param('~vis_rate')
        self.final_wpt_scale = rospy.get_param('~final_wpt_scale')
        self.tl_marker_scale = rospy.get_param('~tl_marker_scale')

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
                    '/visualization_basewp_path', Path, queue_size=1,
                    latch=True)

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
            self.basewp_poses.append(base_wp_msg.waypoints[wp_idx].pose)
        self.subs['/base_waypoints'].unregister()

        # Publish base waypoint path for RViz
        basewp_header = Header()
        basewp_header.frame_id = "/world"
        basewp_header.stamp = rospy.Time()
        basewp_path = Path()
        basewp_path.header = basewp_header
        basewp_path.poses = self.basewp_poses
        self.pubs['/visualization_basewp_path'].publish(basewp_path)

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

                # Final Waypoint Line
                final_wp_line = Marker()
                final_wp_line.id = 1
                final_wp_line.header.frame_id = "/world"
                final_wp_line.header.stamp = rospy.Time()
                final_wp_line.type = Marker.LINE_STRIP
                final_wp_line.action = Marker.ADD
                final_wp_line.scale.x = self.final_wpt_scale
                final_wp_line.scale.y = self.final_wpt_scale
                final_wp_line.scale.z = self.final_wpt_scale
                final_wp_line.color.a = 1.0
                final_wp_line.color.r = 0.0
                final_wp_line.color.g = 1.0
                final_wp_line.color.b = 0.0
                # Make local copy for looping to prevent interrupt by callback
                final_waypoints = deepcopy(self.final_waypoints)
                for wp_idx in range(len(final_waypoints)):
                    final_wp_line.points.append(
                                    final_waypoints[wp_idx].pose.pose.position)
                marker_array.markers.append(final_wp_line)

                # Traffic Waypoint Spheres
                tl_marker = Marker()
                tl_marker.id = 2
                tl_marker.header.frame_id = "/world"
                tl_marker.header.stamp = rospy.Time()
                tl_marker.type = Marker.SPHERE_LIST
                tl_marker.action = Marker.ADD
                tl_marker.scale.x = self.tl_marker_scale
                tl_marker.scale.y = self.tl_marker_scale
                tl_marker.scale.z = self.tl_marker_scale
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
                det_tl_marker = Marker()
                det_tl_marker.id = 3
                det_tl_marker.header.frame_id = "/world"
                det_tl_marker.header.stamp = rospy.Time()
                if (len(self.base_waypoints) > 0
                        and self.traffic_waypoint >= 0
                        and self.traffic_waypoint < len(self.base_waypoints)):
                    det_tl_marker.type = Marker.CUBE
                    det_tl_marker.action = Marker.ADD
                    det_tl_marker.scale.x = self.tl_marker_scale
                    det_tl_marker.scale.y = self.tl_marker_scale
                    det_tl_marker.scale.z = self.tl_marker_scale
                    det_tl_marker.color.a = 1.0
                    det_tl_marker.color.r = 1.0
                    det_tl_marker.color.g = 0.0
                    det_tl_marker.color.b = 0.0
                    tl_idx = self.traffic_waypoint
                    # Make local copy to modify z position
                    tl_pose = deepcopy(self.base_waypoints[tl_idx].pose.pose)
                    tl_pose.position.z += 20  # offset the marker above waypnts
                    det_tl_marker.pose = tl_pose
                else:
                    det_tl_marker.action = Marker.DELETE
                marker_array.markers.append(det_tl_marker)

                # Publish final array of markers for RViz
                self.pubs['/visualization_marker_array'].publish(marker_array)

            self.rate.sleep()


if __name__ == '__main__':
    VisNode()
