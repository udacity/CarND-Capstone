#!/usr/bin/env python

import rospy
from styx_msgs.msg import TrafficLight, TrafficLightArray
from visualization_msgs.msg import Marker, MarkerArray

'''
This node publishes the traffic lights from the /vehicle/traffic_lights topic.
'''

def trafficLightToMarker(light, frame_id, ts=rospy.Time(0), idx=0):
    scale = 25
    color = [0.7, 0.7, 0.7] # grey
    if light.state == TrafficLight.GREEN:
        color = [0.0, 1.0, 0.0]
    if light.state == TrafficLight.YELLOW:
        color = [1.0, 1.0, 0.0]
    if light.state == TrafficLight.RED:
        color = [1.0, 0.0, 0.0]

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = ts
    marker.id = idx
    marker.type = Marker.SPHERE
    marker.pose.position = light.pose.pose.position
    marker.pose.orientation = light.pose.pose.orientation
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.a = 1
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    return marker


class TrafficLightVisualizer(object):
    def __init__(self):
        rospy.init_node('traffic_light_visualizer')

        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.get_traffic_light_status_cb)

        self.traffic_light_marker_pub = rospy.Publisher('traffic_light_markers', MarkerArray, queue_size=1)

        rospy.spin()

    def get_traffic_light_status_cb(self, msg):
        array = MarkerArray()

        for i, light in enumerate(msg.lights):
            array.markers.append(trafficLightToMarker(light, 'world', ts=rospy.Time.now(), idx=i))

        self.traffic_light_marker_pub.publish(array)


if __name__ == '__main__':
    try:
        TrafficLightVisualizer()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
