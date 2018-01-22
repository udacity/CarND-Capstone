#!/usr/bin/env python

import time
import math
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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
"""
We assume:
  - Distance between consecutive waypoints ~ 0.5 to 1m
  - Max speed ~ 10 MPH = 4.47 m/s

PUBLISH_RATE is set to 20 Hz. Under this conditions, the car shouldn't move more than 1-2 waypoints between consecutive calls.
"""
# Publishing rate (Hz).
PUBLISH_RATE = 20

# Max waypoint distance we admit for a local minimum (m).
max_local_distance = 20.0
# Force publishing if next traffic light changes.
publi_light_change = True
update_wp_cur_pose = True
debug = True

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        #rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # Subscribers and publishers are added.
        self.base_wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size = 1)
        
        # TODO: Add other member variables you need below
        # Variables are added.
        # List of waypoints received from /base_waypoints.
        self.base_waypoints = []
        # Original speeds of the waypoints.
        self.base_wp_orig_v = []
        # Next waypoint in the same direction as the vehicle.
        self.next_waypoint = None
        # Car position.
        self.current_pose = None
        # Index of the waypoint of the next red light.
        self.red_light_waypoint = None
        # Message sequence number /final_waypoints.
        self.msg_seq = 0
        
        # Parameters
        # The stop is activated or deactivated with the red light.
        self.stop_on_red = rospy.get_param('~stop_on_red', True)
        # The stop is activated or deactivated with the last waypoint.
        self.force_stop_on_last_waypoint = rospy.get_param('~force_stop_on_last_waypoint', True)
        self.unsubscribe_base_wp = rospy.get_param('/unregister_base_waypoints', False)
		# Brake activation on target.
        self.accel = rospy.get_param('~target_brake_accel', -1.0)
        # Distance in meters where the car will stop before the red light.
        self.stop_distance = rospy.get_param('~stop_distance', 5.0)
        
        # Ensures that the brake is at most half the DBW limit (for braking is feasible using "max" because the deceleration is negative).
        try:
            self.accel = max(rospy.get_param('/dbw_node/decel_limit') / 2, self.accel)
        
        except KeyError:
            pass
        
        # Periodic publications are made in /final_waypoints.
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            self.update_and_publish()
            rate.sleep()
        
        #rospy.spin()
    
    
    def update_and_publish(self):
        # Update next_waypoint based on current_pose and base_waypoints, generate the list of the next LOOKAHEAD_WPS waypoints, update velocity for them, publish them to /final_waypoints.
        
        # 1. Find next_waypoint based on ego position & orientation.
        # Update next_waypoint based on base_waypoints and current_pose.
        if not self.base_waypoints:
            #rospy.logwarn("Waypoints not updated: base_waypoints not available yet.")
            update_wp_cur_pose = False
        
        else:
            if not self.current_pose:
                #rospy.logwarn("Waypoints not updated: current_pose not available yet.")
                update_wp_cur_pose = False
            
            else:
                # Get ego car variables.
                ego_x = self.current_pose.position.x
                ego_y = self.current_pose.position.y
                ego_theta = math.atan2(self.current_pose.orientation.y, self.current_pose.orientation.x)
                
                # If we do have a next_waypoint, we start looking from it, and we stop looking as soon as we get a local minimum. Otherwise we do a full search across the whole track.
                t = time.time()
                wp = None
                yaw = 0
                dist = 1000000 # Long number.
                
                if self.next_waypoint:
                    idx_offset = self.next_waypoint
                    full_search = False
                
                else:
                    idx_offset = 0
                    full_search = True
                
                num_base_wp = len(self.base_waypoints)
                for i in range(num_base_wp):
                    idx_i = (i + idx_offset)%(num_base_wp)
                    wp_x = self.base_waypoints[idx_i].pose.pose.position.x
                    wp_y = self.base_waypoints[idx_i].pose.pose.position.y
                    wp_d = math.sqrt((ego_x - wp_x)**2 + (ego_y - wp_y)**2)
                    
                    if wp_d < dist:
                        dist = wp_d
                        wp = idx_i
                        
                        if debug:
                            # Angle between car heading and waypoint heading.
                            yaw = math.atan2(wp_y - ego_y, wp_x - ego_x) - ego_theta
                    
                    elif not full_search:
                        # Local minimum. If the waypoint makes sense, just use it and break.
                        if dist < max_local_distance:
                            # We found a point.
                            break;
                        
                        else:
                            # We seem to have lost track. We search again.
                            rospy.logwarn("Waypoint updater lost track (local min at %.1f m after %d waypoints). Going back to full search.", dist, i+1)
                            full_search = True
                
                if debug:
                    rospy.loginfo("New next wp [%d] -> (%.1f,%.1f) after searching %d points in %fs", wp, dist * math.cos(yaw), dist * math.sin(yaw), i, time.time() - t)
                
                if wp is None:
                    rospy.logwarn("Waypoint updater did not find a valid waypoint.")
                    update_wp_cur_pose = False
                
                else:
                    self.next_waypoint = wp
                    update_wp_cur_pose = True
        
        # Check if the waypoint of the current position has been updated.
        if update_wp_cur_pose:
            # 2. Generate the list of next LOOKAHEAD_WPS waypoints
            num_base_wp = len(self.base_waypoints)
            last_base_wp = num_base_wp - 1
            waypoint_idx = [idx % num_base_wp for idx in range(self.next_waypoint, self.next_waypoint + LOOKAHEAD_WPS)]
            final_wps = [self.base_waypoints[wp] for wp in waypoint_idx]
            
            # 3. If there is a red light ahead, update velocity for them
            if self.stop_on_red:
                # Start from original velocities
                for idx_wp in waypoint_idx:
				    # Restore original velocities of points
                    self.set_waypoint_velocity(self.base_waypoints, idx_wp, self.base_wp_orig_v[idx_wp])
                
                try:
                    red_idx = waypoint_idx.index(self.red_light_waypoint)
                    # Decelerates within a waypoint list so that it can stop at red_idx.
                    if red_idx <= 0:
                        pass
                    
                    else:
                        dist = self.distance(final_wps, 0, red_idx)
                        step = dist / red_idx
                        # Generate waypoint velocity by traversing the waypoint list backwards:
                        #  - Everything beyond red_idx will have velocity = 0.
                        #  - Before that, constant (de)cceleration is applied until reaching previous waypoint velocity.
                        # We assume constant distance between consecutive waypoints for simplicity.
                        v = 0.0
                        d = 0.0
                        for idx_dis in reversed(range(len(final_wps))):
                            if idx_dis < red_idx:
                                d += step
                                if d > self.stop_distance:
                                     v = math.sqrt(2 * abs(self.accel) * (d - self.stop_distance))
                            
                            if v < self.get_waypoint_velocity(final_wps, idx_dis):
                                self.set_waypoint_velocity(final_wps, idx_dis, v)
                        
                except ValueError:
                    # No red light available: self.red_light_waypoint is None or not in final_waypoints.
                    red_idx = None
                
                if debug:
                    v = self.get_waypoint_velocity(final_wps, 0)
                    rospy.loginfo("Target velocity: %.1f, RL:%s wps ahead.", v, str(red_idx))
            
            # 3b. If we are close to the end of the circuit, make sure that we stop there.
            if self.force_stop_on_last_waypoint or self.base_wp_orig_v[-1] < 1e-5:
                try:
                    last_wp_idx = waypoint_idx.index(last_base_wp)
                    # Decelerates within a waypoint list so that it can stop at last_wp_idx.
                    if last_wp_idx <= 0:
                        pass
                    
			        else:
                        dist = self.distance(final_wps, 0, last_wp_idx)
                        step = dist / last_wp_idx
                        # Generate waypoint velocity by traversing the waypoint list backwards:
                        #  - Everything beyond last_wp_idx will have velocity = 0.
                        #  - Before that, constant (de)cceleration is applied until reaching previous waypoint velocity.
                        # We assume constant distance between consecutive waypoints for simplicity.
                        v = 0.0
                        d = 0.0
                        for idx_dis in reversed(range(len(final_wps))):
                            if idx_dis < last_wp_idx:
                                d += step
                                if d > self.stop_distance:
                                    v = math.sqrt(2 * abs(self.accel) * (d - 0))
                            
                            if v < self.get_waypoint_velocity(final_wps, idx_dis):
                                self.set_waypoint_velocity(final_wps, idx_dis, v)
                
                except ValueError:
                    # Last waypoint is not one of the next LOOKAHEAD_WPS.
                    pass
            
            # 4. Publish waypoints to "/final_waypoints".
            wp_msg = Lane()
            wp_msg.header.seq = self.msg_seq
            wp_msg.header.stamp = rospy.Time.now()
            wp_msg.header.frame_id = '/world'
            wp_msg.waypoints = final_wps
            self.final_waypoints_pub.publish(wp_msg)
            self.msg_seq += 1
    
    
    def pose_cb(self, msg):
        # TODO: Implement
        #pass
        # The ego pose is received and stored
        self.current_pose = msg.pose
    
    
    #def waypoints_cb(self, waypoints):
    def waypoints_cb(self, msg):
        # TODO: Implement
        #pass
        # Receives and stores the complete list of waypoints.
        t = time.time()
        wps = msg.waypoints
        num_wp = len(wps)
        
        if self.base_waypoints and self.next_waypoint is not None:
            # Normally we assume that waypoint list doesn't change or at least, not in the position where the car is located. If that happens, just handle it.
            # Compare two waypoints to see if they are the same.
            max_d = 0.5 # Distance
            max_v = 0.5 # Velocity
            
            dl = lambda a, b: math.sqrt((a.x  - b.x)**2 + (a.y - b.y)**2  + (a.z - b.z)**2)
            ddif = dl(self.base_waypoints[self.next_waypoint].pose.pose.position, waypoints[self.next_waypoint].pose.pose.position)
            if ddif < max_d:
                is_same_wp = True
            
            else:
                is_same_wp False
            
            if not is_same_wp:
                # We can't assume to know the previous waypoint.
                self.next_waypoint = None
                # Just to debug. It will be updated later.
                self.base_waypoints = None
                rospy.logwarn("Base waypoint list changed.")
        
        else:
            # There's no change. We could probably come back here.
            pass
        
        # Uncomment for debug.
        # Stamp waypoint index in PoseStamped and TwistStamped headers of internal messages.
        #for idx in range(len(waypoints)):
        #    waypoints[idx].pose.header.seq = idx
        #    waypoints[idx].twist.header.seq = idx
        
        self.base_wp_orig_v = [self.get_waypoint_velocity(waypoints, idx) for idx in range(num_wp)]
        
        if debug and not self.base_waypoints:
            dist = self.distance(waypoints, 0, num_wp - 1)
            rospy.loginfo("Received: %d waypoints, %.1f m, %.1f m/wp in t=%f", num_wp, dist, dist / num_wp, time.time() - t)
        
        self.base_waypoints = waypoints
        
        if self.unsubscribe_base_wp:
            self.base_wp_sub.unregister()
    
    
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        #pass
        # The waypoint index is received and stored for the next traffic light in red. If the index is < 0, then there is no red traffic light ahead.
        prev_red_light_wp = self.red_light_waypoint
        self.red_light_waypoint = msg.data if msg.data >= 0 else None
        
        if prev_red_light_wp != self.red_light_waypoint:
            if debug:
                rospy.loginfo("Traffic light changed: %s", str(self.red_light_waypoint))
            
            # Is updated and published if the following traffic light has changed.
            if publi_light_change:
                self.update_and_publish()
    
    
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        # In this section there is nothing to do, the obstacle control is not necessary in this version.
        pass
    
    
    #def get_waypoint_velocity(self, waypoint):
    #    return waypoint.twist.twist.linear.x
    def get_waypoint_velocity(self, wps, waypoint):
        return wps[waypoint].twist.twist.linear.x
    
    
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity
    
    
    def distance(self, waypoints, wp1, wp2):
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