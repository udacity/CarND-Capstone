#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
import Utility
import math
import time
import numpy as np

from threading import Lock

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

'''
Notes:
The Waypoint position information is contained in the Pose position and orientation of the 
Lane data structure. The rest of the structures appear to be blanked out except for twist - linear velocity
- x which sometimes has a value set

Steps:
1 Get the current vehicle location wrt to the provided 'map' or waypoints
2 Create a simple trajectory that will move the vehicle along the path to the next waypoint
3 Publish this next trajectory set. (after the vehicle pose comes in)
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

#class Vehicle(object):
#    pose
#    
#    def __init(self):
#        pass
#        
#    def update_pose(self, msg):
#        # Set the pose - PoseStamped.Pose
#        self.pose = msg.pose
#        
#    def position_xyz(self):
#        return self.pose.position.x,self.pose.position.y,self.pose.position.z
#    
#    def position_frenet(self):
#        pass
            
        
        

class WaypointUpdater(object):
#    self.vehicle = Vehicle()
    waypoints = None
    pose = None
    prev_pose = None
    heading = None
    s = None
    d = None
    map_x = None
    map_y = None
    map_s = None
    generated_waypoints = []
    minIdx = 0
    mutex = Lock()
    traffic_light_map = None
    
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /Lantraffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.seq = 0

        # TODO: Add other member variables you need below

        #rospy.spin()
        self.loop()

    def pose_cb(self, msg):
        self.prev_pose = self.pose
        self.pose = msg.pose
        quaternion = (
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w)
        self.heading = Utility.getHeading(quaternion)
        
    def ddDistance(self,x1,y1,x2,y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
    
    def findNextWaypoint(self):
        '''
        Get the closes waypoint to the vehicle
        '''
        index = 0
        minD = 100000
        count = 0
        idx = 0
        for wp in self.waypoints:
            #d = self.ddDistance(self.pose.position.x,self.pose.position.y,wp.pose.pose.position.x,wp.pose.pose.position.y)
            d = self.ddDistance(self.pose.position.x,0,wp.pose.pose.position.x,0)# Only do X distance
            if d < minD:
                minD = d
                index = count
            count += 1
        return index
        
    
    def methodA(self):
        start = time.time()
        self.s, self.d = Utility.convertToFrenet(self.pose.position.x, self.pose.position.y, self.heading, self.map_x, self.map_y)
        pts = []
        for i in range(LOOKAHEAD_WPS):
            ns = self.s + 0.75* i
            nd = self.d
            x,y = Utility.convertFromFrenet(ns, nd, self.map_s, self.map_x, self.map_y)
            wp = Waypoint()
            wp.pose.pose.position.x = x
            wp.pose.pose.position.y = y
            wp.pose.header.seq = i
            wp.pose.header.stamp = rospy.Time(0)
            wp.pose.header.frame_id = '/world'
            wp.twist.header.seq = i
            wp.twist.header.stamp = rospy.Time(0)
            wp.twist.header.frame_id = '/world'
            minIdx = Utility.nextWaypoint(x, y,self.heading, self.map_x, self.map_y)
            wp.pose.pose.orientation = self.waypoints[(minIdx)].pose.pose.orientation # Set the orientation to that of the nearest wp
            #wp.twist.twist.linear.x = 15
            pts.append(wp)
            # Set the velocity of the newly loaded waypoint
            self.set_waypoint_velocity(pts,i,5)
            rospy.loginfo('@_2 waypoint_%s: (X,Y) (%s,%s) next idx %s heading %s s %s d %s', str(i), str(x), str(y), str(minIdx), self.heading, self.s, self.d) 
        self.generated_waypoints = pts

    def methodB(self):
        start = time.time()
        newIdx = Utility.nextWaypoint(self.pose.position.x, self.pose.position.y,self.heading, self.map_x, self.map_y, last_idx=None)
        #newIdx = self.findNextWaypoint()
        end = time.time()
        newIdx += 2
        newIdx %= len(self.map_x)
        rospy.loginfo('@_2 C exeTime %s X,Y (%s,%s) MX,My (%s,%s) NextIDX %s LenWaypoints %s heading %s distance %s', 
        str(end - start), 
        str(self.pose.position.x),
        str(self.pose.position.y), 
        str(self.waypoints[newIdx].pose.pose.position.x),
        str(self.waypoints[newIdx].pose.pose.position.y), 
        str(self.minIdx), 
        str(len(self.waypoints)), 
        self.heading, 
        self.distance(self.waypoints,newIdx,self.minIdx) )
        self.minIdx = newIdx
        
        if True:
            self.generated_waypoints = []
            for wp in range(LOOKAHEAD_WPS):
                self.generated_waypoints.append(self.waypoints[(self.minIdx+wp)%len(self.waypoints)])
                # Cross track error causes the target velocity to decrease (does not recover)
                if self.minIdx > 6000 and self.minIdx < 7000:
                    self.set_waypoint_velocity(self.generated_waypoints,wp,10)
                else:
                    self.set_waypoint_velocity(self.generated_waypoints,wp,20)
        else:
            # Fill in the data first time around
            if len(self.generated_waypoints) < 1:                            
                for wp in range(LOOKAHEAD_WPS):
                    self.generated_waypoints.append(self.waypoints[self.minIdx+wp%len(self.waypoints)])
            else:
                process = True
                while process:
                    # This means that the real next waypoint is not the one we are publishing
                    if len(self.generated_waypoints) == 0:
                        break
                    if self.waypoints[self.minIdx].pose.pose.position.x != self.generated_waypoints[0].pose.pose.position.x:
                        self.generated_waypoints.pop(0)
                    else:
                        process = False
                        
                for wp in range(len(self.generated_waypoints),LOOKAHEAD_WPS):
                    self.generated_waypoints.append(self.waypoints[(self.minIdx+wp)%len(self.waypoints)])
                    
    def methodC(self):
        start = time.time()
        
        self.s, self.d = Utility.convertToFrenet(
        self.pose.position.x, 
        self.pose.position.y, 
        self.heading, 
        self.map_x, 
        self.map_y)
        
        # Setup the Coordinates to get the spline between
        px = []
        py = []
        if Utility.distance(self.prev_pose.position.x, self.prev_pose.position.y,
        self.pose.position.x, self.pose.position.y) > 15:
            px.append(self.prev_pose.position.x)
            py.append(self.prev_pose.position.y)
        px.append(self.pose.position.x)
        py.append(self.pose.position.y)
        
        # Get the next waypoint
        newIdx = Utility.nextWaypoint(
        self.pose.position.x, 
        self.pose.position.y,
        self.heading, 
        self.map_x, 
        self.map_y, 
        last_idx=None)

        # Add N future waypoints to the match list
        for i in range(80):
            idx = newIdx + 15 + i * 5
            idx %= len(self.map_x)
            px.append(self.map_x[idx])
            py.append(self.map_y[idx])

        rospy.loginfo( '@_2 nextIdx %s heading %s X %s Y %s %s %s',newIdx, self.heading, self.pose.position.x, self.pose.position.y, px, py )
        # Fit the spline
        tkc = Utility.getSplineCoeffs(px,py)
        
        pts = []
        
        s = np.arange(0, tkc.s[-1], 0.5)
        index = 0
        for i in s:
            ix,iy = Utility.fitX(i,tkc)
            wp = Waypoint()
            # Put in the orientation of the waypoint
            
            wp.pose.pose.orientation = Quaternion(*Utility.getQuaternion(0,0,tkc.calc_yaw(i)))
            wp.pose.pose.position.x = ix
            wp.pose.pose.position.y = iy
            wp.pose.header.seq = i
            wp.pose.header.stamp = rospy.Time(0)
            wp.pose.header.frame_id = '/world'
            wp.twist.header.seq = i
            wp.twist.header.stamp = rospy.Time(0)
            wp.twist.header.frame_id = '/world'
            pts.append(wp)
            self.set_waypoint_velocity(pts,index,20)
            index += 1
            if index > range(LOOKAHEAD_WPS):
                break

        self.generated_waypoints = pts

    def loop(self):
        rate = rospy.Rate(1) # N Hz
        while not rospy.is_shutdown():
            if self.waypoints and self.pose and self.prev_pose:
                if False:
                    self.methodC()
                else:
                    self.methodB()
                # Create a new lane message type
                l = Lane()
                l.header.frame_id = '/world'
                l.header.stamp = rospy.Time(0)
                l.waypoints = self.generated_waypoints
                
                # Publish the generated message
                self.final_waypoints_pub.publish(l)
                self.debug_traffic_lights()
            rate.sleep()

    def waypoints_cb(self, waypoints):
        # Update / set the current waypoints
        self.waypoints = waypoints.waypoints
        
        # Fill in the local map data 
        if self.map_x is None:
            self.map_x = []
            self.map_y = []
            # Pull out the x and y waypoints
            for wp in self.waypoints:
                self.map_x.append(wp.pose.pose.position.x)
                self.map_y.append(wp.pose.pose.position.y)
            
            # Generate the s map
            self.map_s = Utility.generateMapS(self.map_x,self.map_y)
            
            # Convert to numpy arrays
            #self.map_x = np.asarray(self.map_x)
            #self.map_s = np.asarray(self.map_s)
            #self.map_y = np.asarray(self.map_y)
        

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

    def eucld_distance(self, pnta, pntb):
        return math.sqrt((pnta.x-pntb.x)**2 + (pnta.y-pntb.y)**2  + (pnta.z-pntb.z)**2)
        
    def distance(self, waypoints, wp1, wp2):
        '''
        Get the distance between waypoints indexed by wp1 and wp2.
        This returns the sum of the euclidean distance between
        all intermediate waypoints
        '''
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
