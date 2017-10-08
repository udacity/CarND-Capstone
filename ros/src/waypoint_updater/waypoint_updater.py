#!/usr/bin/env python

import rospy
import datetime
import tf.transformations
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TwistStamped
from std_msgs.msg import Int32, Float32, Bool
from styx_msgs.msg import Lane, Waypoint

import stop_planner
import statemachine
import numpy as np
import yaml

import math

MAX_DECEL = 5.0
MAX_ACCEL = 1.0

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

LOOKAHEAD_WPS = 240 # Number of waypoints we will publish. 
# tested at speeds up to 115 mph


# takes geometry_msgs/Point
def point_dist_sq(a, b):
    dx = a.x-b.x
    dy = a.y-b.y
    dz = a.z-b.z
    return dx*dx+dy*dy+dz*dz

# takes geometry_msgs/Point
def point_dist(a, b):
    return math.sqrt(point_dist_sq(a, b))

# takes styx_msgs/Waypoint
# returns geometry_msgs/Point
def waypoint_to_point(wp):
    point = wp.pose.pose.position
    return point

# takes x and y position (floats)
# returns geometry_msgs/Pose
def point_to_pose(x, y):
    pt = Point()
    pt.x = x
    pt.y = y
    pose = Pose()
    pose.position = pt
    return pose

# takes styx_msgs/PoseStamp
# returns geometry_msgs/Point
def pose_to_point(pose):
    point = pose.pose.position
    return point

# takes geometry_msgs/Point 
# returns tuple of floats 
def waypoints_to_vec(a, b):
    return (b.x-a.x, b.y-a.y)

def dot_prod(a, b):
    return a[0]*b[0]+a[1]*b[1]

# takes 3 geometry_msgs/Point 
# returns float ratio of distance that b,
# projected onto line ac, is between
# a and c (ratio is 0. if b is at a, and
# 1. if b is at c
def line_ratio(a, b, c):
    ac = waypoints_to_vec(a, c)
    ab = waypoints_to_vec(a, b)
    bc = waypoints_to_vec(b, c)
    da = dot_prod(ab, ac)
    # dc = dot_prod(bc, ac)
    ac_dist = point_dist(a, c)
    return da/ac_dist



class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        #rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_waypoint_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # But there is no /obstacle_waypoint topic, or Obstacle Detection node
        # rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.next_waypoint_pub = rospy.Publisher('next_waypoint', Int32, queue_size=1)
        self.tl_distance_pub = rospy.Publisher('tl_distance', Float32, queue_size=1)
        self.go_to_stop_state_pub = rospy.Publisher('go_to_stop_state', Bool, queue_size=1)
        self.wps = []
        self.wp_ss = []
        # self.full_wps = []
        self.prev_pt = Point()
        self.prev_index = -1
        self.next_pt = -1
        self.stopPlanner = None
        self.tl_distance = -1
        self.test_tl = False
        self.current_velocity = None
        self.T = 3.
        self.final_wps = []

        self.tl_distance = -1
        self.prev_tl_distance = -1
        self.red_tl = True
        self.red_tl_prev = True
        self.tl_count = 0
        self.stopped = False
        self.started = False    

        self.pose_set = False
        self.prev_pose = None
        self.accel_wps = []
        self.decel_wps = []

        self.velocity = rospy.get_param('velocity') * 1000. / (60. * 60.)
        ## calculate stopping distance based on allowed max velocity
        # u = 0.35 = friction coefficient
        # t = 1.5 = brake time
        # g = 9.8 = force due to earth gravity
        self.stopping_distance = (self.velocity * 1.5) + (self.velocity**2 / (2. * 0.35 * 9.8))


        ## get stop lines positions from paramenter
        self.stop_lines = yaml.load(rospy.get_param("/traffic_light_config"))['stop_line_positions']
        ## state table with transitions 
        self.fsm = statemachine.FSM({'stopped':[
                                                ['stopped', 'conditions', self.stopped_cb], 
                                                ['start_moving_cb', 'conditions', self.start_moving_cb]
                                                ],
                                    'start_moving_cb': [
                                                        ['start_moving_cb', 'conditions', self.start_moving_cb],
                                                        ['moving', 'conditions', self.moving_cb]
                                                        ],
                                    'moving': [
                                                ['moving', 'conditions', self.moving_cb], 
                                                ['slow_down', 'conditions', self.slow_down_cb]
                                                ],
                                    'slow_down': [
                                                ['slow_down', 'conditions', self.slow_down_cb],
                                                ['go_to_stop', 'conditions', self.go_to_stop_cb]
                                                ],
                                    'go_to_stop': [
                                                    ['go_to_stop', 'conditions', self.go_to_stop_cb], 
                                                    ['stopped', 'conditions', self.stopped_cb], 
                                                    ['start_moving_cb', 'conditions', self.start_moving_cb]]},
                                    curstate='go_to_stop')

        

        
        # TODO: Add other member variables you need below

        rospy.spin()

    def slow_down_cb(self):
        '''
        In this state the car will start to slow its velocity when aproaching to the tl
        '''
        if self.distance_to_tl > self.stopping_distance:
            self.go_to_stop_state_pub.publish(False)
            olane = Lane()
            olane.header.frame_id = '/world'
            olane.header.stamp = rospy.Time(0)
            next_tl = self.get_next_tl(self.next_pt)
            if self.next_pt == next_tl-1:
                next_tl += 1
            #olane.waypoints=self.wps[self.next_pt:next_tl-1][:]
            waypoints = self.decelerate(self.next_pt, next_tl, self.current_velocity * 0.7)  # deceleraate upto 70% of current velocity
            olane.waypoints = waypoints
            self.decel_wps = waypoints
            # Handle case where we are near the end of the track;
            # add points at the beginning of the track
            # if past_zero_pt > 0:
            #     olane.waypoints.extend(self.wps[:past_zero_pt])
            # rospy.loginfo('[go_to_stop] velocity: %f', olane.waypoints[0].twist.twist.linear.x)
            self.final_waypoints_pub.publish(olane)
            self.next_waypoint_pub.publish(self.next_pt)
            return True
        else:
            self.decel_wps = []
        return False
        

    def start_moving_cb(self):
        '''
        In this state the car will go from stoped  to moving at tl
        '''
        #rospy.logwarn("Entering moving_cb !!!")
        if self.current_velocity < (self.velocity * .8):
            self.go_to_stop_state_pub.publish(False)
            olane = Lane()
            olane.header.frame_id = '/world'
            olane.header.stamp = rospy.Time(0)
            sz = len(self.wps)
            wpsz = len(self.wps)
            end_pt = self.next_pt+LOOKAHEAD_WPS
            past_zero_pt = end_pt - wpsz
            end_pt = min(end_pt, wpsz)
            waypoints=self.wps[self.next_pt:end_pt][:]
            # Handle case where we are near the end of the track;
            # add points at the beginning of the track
            if past_zero_pt > 0:
                waypoints.extend(self.wps[:past_zero_pt][:])
            waypoints = self.accelerate(self.next_pt, end_pt)
            olane.waypoints = waypoints
            self.accel_wps = waypoints
            #rospy.loginfo('[start_moving_cb] velocity: %f', olane.waypoints[0].twist.twist.linear.x)
            self.final_waypoints_pub.publish(olane)
            self.next_waypoint_pub.publish(self.next_pt)
            return True
        else:
            self.accel_wps = []
        return False

        
    def moving_cb(self):
        '''
        In this state the car will move at full speed
        '''
        if self.distance_to_tl > 100:
            self.go_to_stop_state_pub.publish(False)
            olane = Lane()
            olane.header.frame_id = '/world'
            olane.header.stamp = rospy.Time(0)
            sz = len(self.wps)
            wpsz = len(self.wps)
            end_pt = self.next_pt+LOOKAHEAD_WPS
            past_zero_pt = end_pt - wpsz
            end_pt = min(end_pt, wpsz)
            olane.waypoints=self.wps[self.next_pt:end_pt][:]
            # Handle case where we are near the end of the track;
            # add points at the beginning of the track
            if past_zero_pt > 0:
                olane.waypoints.extend(self.wps[:past_zero_pt][:])
            #rospy.loginfo('[moving_cb] velocity: %f', olane.waypoints[0].twist.twist.linear.x)
            for i in range(len(olane.waypoints)):
                olane.waypoints[i].twist.twist.linear.x = self.velocity
            self.final_waypoints_pub.publish(olane)
            self.next_waypoint_pub.publish(self.next_pt)
            return True
        return False

    def stopped_cb(self):
        '''
        In this state the car will stop or remain stop at tl
        '''
        if self.red_tl == True:
            self.go_to_stop_state_pub.publish(True)
            self.final_waypoints_pub.publish(Lane())
            self.next_waypoint_pub.publish(self.next_pt)
            return True
        # rospy.loginfo("[stopped_cb] moving to stop line, Traffic Ligh : %s", self.red_tl)
        return False
    
    
    def go_to_stop_cb(self):
        '''
        In this stop the car will reduce speed to stop at tl
        '''
        if self.distance_to_tl > 6 and self.red_tl == True: 
            
            self.go_to_stop_state_pub.publish(True)
            olane = Lane()
            olane.header.frame_id = '/world'
            olane.header.stamp = rospy.Time(0)
            next_tl = self.get_next_tl(self.next_pt)
            if self.next_pt == next_tl-1:
                next_tl += 1
            waypoints=self.wps[self.next_pt:next_tl-1][:]
            waypoints=self.decelerate(self.next_pt, next_tl)
            olane.waypoints = waypoints
            self.decel_wps = waypoints
            # Handle case where we are near the end of the track;
            # add points at the beginning of the track
            # if past_zero_pt > 0:
            #     olane.waypoints.extend(self.wps[:past_zero_pt])
            # rospy.loginfo('[go_to_stop] velocity: %f', olane.waypoints[0].twist.twist.linear.x)
            
            self.final_waypoints_pub.publish(olane)
            self.next_waypoint_pub.publish(self.next_pt)
            return True
        else:
            self.decel_wps = []
        return False

       

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)




    def decelerate(self, next_pt, next_tl, target_velocity=0.):
        '''
        Copy of the function used in waypoing loader to reduce 
        the velocity of the car when apporaching a tl
        '''
        if len(self.decel_wps) > 10:
            return self.decel_wps[1:]

        # last = waypoints[-1]
        # last.twist.twist.linear.x = target_velocity
        # for wp in waypoints[:-1][::-1]:
        #     dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
        #     vel = math.sqrt(2 * MAX_DECEL * dist)
        #     if vel < 1.:
        #         vel = 0.
        #     wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        
        xyz = self.wps[next_pt].pose.pose.position
        q = self.wps[next_pt].pose.pose.orientation
        end_xyz = self.wps[next_tl].pose.pose.position
        end_q = self.wps[next_tl].pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        (endroll, endpitch, end_yaw) = tf.transformations.euler_from_quaternion([end_q.x, end_q.y, end_q.z, end_q.w])
        
        s, d = self.stopPlanner.getFrenet(xyz.x, xyz.y, yaw, self.wps)
        T = 2.0

        #rospy.loginfo("tl_distance: %d, s: % %f" % self.tl_distance, s)
        dist_to_tl = self.stopPlanner.distance(self.wps, next_pt, next_tl) 
        ss = s + dist_to_tl
        if self.current_velocity == 0:
            current_velocity = 1.
        else:
            current_velocity = self.current_velocity
        T = 2. * dist_to_tl / current_velocity
        dt = 0.03
        if T < dt:
            T = dt
        if T > 1.5:
            T = 1.5
        n = T / dt
        if n > LOOKAHEAD_WPS:
            n = LOOKAHEAD_WPS
        elif n < 1.:
            n = 1

        coeff = self.stopPlanner.JMT([s, self.current_velocity, -MAX_DECEL], [ss, 0.0, 0.0], T)
        vcoeff = self.stopPlanner.JMT([self.current_velocity, -MAX_DECEL, 1.0], [0.0, 0.0, 0.0], T)
        fy = np.poly1d(coeff)
        fyv = np.poly1d(vcoeff)
        s_x = np.linspace(0, T, n)
        sss = fy(s_x)
        vvv = fyv(s_x)
        vvv[vvv > self.velocity] = self.velocity
        vvv[vvv < 1.0] = 0.0
        final_path = []
        for i in range(len(sss)):
            d = 0. ## we dont want to change lanes
            px, py = self.stopPlanner.getXY(sss[i], d, self.stopPlanner.map_s, self.wps)
            final_path.append([px, py])
        final_path = np.array(final_path)
        o2lane = Lane()
        o2lane.header.frame_id = '/world'
        o2lane.header.stamp = rospy.Time(0)
        cur_x = xyz.x
        cur_y = xyz.y
        waypoints = []
        for i in range(len(final_path)):
            p = Waypoint()
            p.pose.pose.position.x = final_path[i][0]
            p.pose.pose.position.y = final_path[i][1]
            p.pose.pose.position.z = 0.
            yw = math.atan2(final_path[i][1] - cur_y, final_path[i][0] - cur_x)
            cur_x = final_path[i][0]
            cur_y = final_path[i][1]
            if yw < 0:
                yw = yw + 2 * np.pi
            q = tf.transformations.quaternion_from_euler(0.,0.,yw)
            p.pose.pose.orientation = Quaternion(*q)
            v = vvv[i]
            p.twist.twist.linear.x = float(v)
            waypoints.append(p)
        
        
        
        
        return waypoints




    def accelerate(self, next_pt, end_pt):
        '''
        Used by start_moving state but is not working yet
        '''
        if len(self.accel_wps) > 10:
            return self.accel_wps[1:]
        
        # start = self.next_pt - 1
        # if start < 0:
        #     start = 0
        # #start_pos = self.wps[start]
        # start_pos = waypoints[0]
        # vel = self.current_velocity

        # # #rospy.loginfo("[accelerate] current velocity: %f", vel)
        # for wp in waypoints[:]:
        #     dist = self.distance(wp.pose.pose.position, start_pos.pose.pose.position)
        #     vel = math.sqrt(vel**2 + (2. * MAX_ACCEL * dist))
        #     if vel < 1.0:
        #         vel = 1.0
        #     wp.twist.twist.linear.x = vel
        #     if vel > self.velocity:
        #         vel = self.velocity
        ## JMT 
        # coeff_v = self.JMT([vel, MAX_ACCEL, 1.0], [self.velocity, MAX_ACCEL, 1.0], 5.)
        # fyv = np.poly1d(coeff_v)
        # dist = 0
        # for wp in waypoints[:]:
        #     dist += self.distance(wp.pose.pose.position, start_pos.pose.pose.position)
        #     R = np.roots([2*MAX_ACCEL*dist, 4*vel*dist, -4*dist**2])
        #     T = R[R>0]
        #     if T.size == 0:
        #         T = 0
        #     vfinal = fyv(T)
        #     if vfinal > self.velocity:
        #         vfinal = self.velocity
        #     # elif vfinal < 1.:
        #     #     vfinal = 1.
        #     wp.twist.twist.linear.x = vfinal
        ## JMT FULL
        xyz = self.wps[next_pt].pose.pose.position
        q = self.wps[next_pt].pose.pose.orientation
        end_xyz = self.wps[end_pt].pose.pose.position
        end_q = self.wps[end_pt].pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        (endroll, endpitch, end_yaw) = tf.transformations.euler_from_quaternion([end_q.x, end_q.y, end_q.z, end_q.w])
        
        s, d = self.stopPlanner.getFrenet(xyz.x, xyz.y, yaw, self.wps)
        T = 6.0

        #rospy.loginfo("tl_distance: %d, s: % %f" % self.tl_distance, s)
        ss = s + self.stopPlanner.distance(self.wps, next_pt, end_pt)
        coeff = self.stopPlanner.JMT([s, self.current_velocity, MAX_ACCEL], [ss, self.velocity, MAX_ACCEL], T)
        vcoeff = self.stopPlanner.JMT([self.current_velocity, MAX_ACCEL, 1.0], [self.velocity, MAX_ACCEL, 1.0], T)
        fy = np.poly1d(coeff)
        fyv = np.poly1d(vcoeff)
        n = T / 0.03
        s_x = np.linspace(0, T, n)
        sss = fy(s_x)
        vvv = fyv(s_x)
        final_path = []
        for i in range(len(sss)):
            d = 0. ## we dont want to change lanes
            px, py = self.stopPlanner.getXY(sss[i], d, self.stopPlanner.map_s, self.wps)
            final_path.append([px, py])
        final_path = np.array(final_path)
        o2lane = Lane()
        o2lane.header.frame_id = '/world'
        o2lane.header.stamp = rospy.Time(0)
        cur_x = xyz.x
        cur_y = xyz.y
        waypoints = []
        for i in range(len(final_path)):
            p = Waypoint()
            p.pose.pose.position.x = final_path[i][0]
            p.pose.pose.position.y = final_path[i][1]
            p.pose.pose.position.z = 0.
            yw = math.atan2(final_path[i][1] - cur_y, final_path[i][0] - cur_x)
            cur_x = final_path[i][0]
            cur_y = final_path[i][1]
            if yw < 0:
                yw = yw + 2 * np.pi
            q = tf.transformations.quaternion_from_euler(0.,0.,yw)
            p.pose.pose.orientation = Quaternion(*q)
            v = vvv[i]
            if v > self.velocity:
                v = self.velocity
            p.twist.twist.linear.x = float(v)
            waypoints.append(p)
            
        return waypoints

    # def JMT(self, start, end, T):
    #     """
    #     Calculates Jerk Minimizing Trajectory for start, end and T.
    #     """
    #     a_0, a_1, a_2 = start[0], start[1], start[2] / 2.0
    #     c_0 = a_0 + a_1 * T + a_2 * T**2
    #     c_1 = a_1 + 2* a_2 * T
    #     c_2 = 2 * a_2
        
    #     A = np.array([
    #             [  T**3,   T**4,    T**5],
    #             [3*T**2, 4*T**3,  5*T**4],
    #             [6*T,   12*T**2, 20*T**3],
    #         ])
    #     B = np.array([
    #             end[0] - c_0,
    #             end[1] - c_1,
    #             end[2] - c_2
    #         ])
    #     a_3_4_5 = np.linalg.solve(A,B)
    #     alphas = np.concatenate([np.array([a_0, a_1, a_2]), a_3_4_5])
    #     # reverse coefficients to match order used by np.poly1d
    #     return alphas[::-1]

    '''
    PoseStamped:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
    '''

    def get_next_tl(self, car_wp):
        # Find the closest visible traffic light (if one exists)
        next_tl = -1
        if len(self.stop_lines) == 0:
            return -1
        for i in range(len(self.stop_lines)):
            (x, y, wp) = self.stop_lines[i]
            if wp > car_wp:
                next_tl = wp
                break
        # This happens when the car is past the last signal on the
        # track; in this case, look past the end to the first signal:
        if next_tl < 0:
            next_tl = self.stop_lines[0][2]
        return next_tl

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    # def traffic_waypoint_cb(self, msg):
    #     #self.tl_distance = msg.data
    #     pass

    # takes styx_msgs/PoseStamp
    # returns i value of nearest waypoint in self.wps
    def nearest_waypoint(self, pose):
        ppt = pose_to_point(pose)
        prev_dist = point_dist_sq(ppt, self.prev_pt)
        # tested for speeds up to 115 mph
        wp_ahead = 200
        if prev_dist > .5*self.avg_wp_dist*wp_ahead:
            rospy.logwarn("nearest_waypoint: resetting")
            self.prev_index = -1
        self.prev_pt = ppt
        rg = range(0, len(self.wps))
        if self.prev_index > -1:
            rg = range(max(0, self.prev_index-5), min(len(self.wps), self.prev_index+wp_ahead+1))
        mindist = 0.
        mini = -1
        for i in rg:
            wp = self.wps[i]
            wpt = waypoint_to_point(wp)
            dsq = point_dist_sq(wpt, ppt)
            if mini < 0 or dsq < mindist:
                mini = i
                mindist = dsq
        self.prev_index = mini
        if mini == rg[0] or mini == rg[-1]:
            rospy.logwarn("nearest endpoint at end of range: %d %d %d", mini, rg[0], rg[-1])
            self.prev_index = -1
        return mini

    # takes styx_msgs/PoseStamp
    # returns i value of next waypoint in self.wps
    # ("next" assuming car is traversing the waypoints
    # in increasinge order)
    def next_waypoint(self, pose):
        ept = pose_to_point(pose)
        cur = self.nearest_waypoint(pose)
        nwps = len(self.wps)
        if nwps == 0 or cur < 0 or cur > len(self.wps)-1:
            rospy.logwarn("next_waypoint problem %d %d", len(self.wps), cur)
            return -1
        cpt = waypoint_to_point(self.wps[cur])
        prev = (cur+nwps-1)%nwps
        ppt = waypoint_to_point(self.wps[prev])
        nxt = (cur+1)%nwps
        npt = waypoint_to_point(self.wps[nxt])
        eratio = line_ratio(ppt, ept, npt)
        cratio = line_ratio(ppt, cpt, npt)
        if eratio > cratio:
            cur = nxt
        return cur

   



    def pose_cb(self, msg):
        # TODO: Implement
        # rospy.loginfo("Pose %d %s %s", msg.header.seq, msg.header.stamp, msg.header.frame_id)
        # rospy.loginfo("%s", msg.pose)
        # rospy.loginfo("Pose %s", msg.header)
        # rospy.loginfo("Pose %d", msg.header.seq)

        # Don't process if waypoints are not yet loaded

        #rospy.loginfo(self.fsm.states)

        rospy.logwarn('state machine current state: %s', self.fsm.get_currentState())

        if len(self.wps) == 0:
            return
        seq = msg.header.seq
        if seq%1 != 0:
            return
        q = msg.pose.orientation
        xyz = msg.pose.position

        if self.pose_set == False:
            self.prev_pose = xyz
            self.pose_set = True
        else:
            displacement = point_dist(self.prev_pose, xyz)
            self.prev_pose = xyz
            if displacement <= 1e-2:
                self.stopped = True
            else:
                self.stopped = False
            #rospy.loginfo("stopped: %s displacement: %f", self.stopped, displacement)

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        ts = msg.header.stamp.secs + 1.e-9*msg.header.stamp.nsecs
        dtime = datetime.datetime.fromtimestamp(ts)
        dts = dtime.strftime("%H:%M:%S.%f")
        # near_pt is only used for testing
        # near_pt = self.nearest_waypoint(msg)
        next_pt = self.next_waypoint(msg)
        self.next_pt = next_pt
        near_pt = 0
        if next_pt < 0:
            return
        if seq%5 == 0:
            pass
        
        next_tl = self.get_next_tl(next_pt)
        rospy.loginfo('next traffic light: %d', next_tl)
        if next_tl == -1:
            return
        self.distance_to_tl = self.stopPlanner.distance(self.wps, next_pt, next_tl)
        self.fsm.run()

        

    '''
    styx_msgs/Lane:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    styx_msgs/Waypoint[] waypoints
      geometry_msgs/PoseStamped pose
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        geometry_msgs/Pose pose
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
      geometry_msgs/TwistStamped twist
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        geometry_msgs/Twist twist
          geometry_msgs/Vector3 linear
            float64 x
            float64 y
            float64 z
          geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z
    '''

    def waypoints_cb(self, waypoints):
        # rospy.loginfo("Waypoints", waypoints)
        # rospy.loginfo("Waypoints %d\n%s", len(waypoints.waypoints), waypoints.waypoints[0])
        # rospy.loginfo("Waypoints\n%s", waypoints.header)

        # TODO: Implement
        if len(waypoints.waypoints) == len(self.wps):
            # rospy.loginfo("Waypoints: same as before")
            return
        
        self.wps = waypoints.waypoints

        s = 0.
        prev_pt = waypoint_to_point(self.wps[0])
        for i in range(len(self.wps)+1):
            cur_pt = waypoint_to_point(self.wps[i%(len(self.wps))])
            d = point_dist(prev_pt, cur_pt)
            s += d
            # if i < 10 or i > len(self.wps)-10:
            #     print(i, s)
            self.wp_ss.append(s)
            prev_pt = cur_pt

        self.avg_wp_dist = 0.
        for i in range(1, len(self.wps)):
            pt = waypoint_to_point(self.wps[i])
            ppt = waypoint_to_point(self.wps[i-1])
            d = point_dist(pt, ppt)
            self.avg_wp_dist += d
        self.avg_wp_dist /= len(self.wps) - 1

        self.stopPlanner = stop_planner.StopPlanner()
        self.stopPlanner.getMap_s(self.wps)
        #rospy.loginfo("size of map_s: %d " % len(self.stopPlanner.map_s))


        rospy.loginfo("Waypoints: now have %d avg dist %f", len(self.wps), self.avg_wp_dist)

        # set traffic light ids

        for pos in self.stop_lines:
            # Each position is just a two-element list containing
            # the x and y of the stop line
            (x, y) = pos

            # Find the id of the nearest waypoint to this position
            nearest = self.stopPlanner.ClosestWaypoint(x,y,self.wps)
            # print("nearest", nearest)

            # Add the waypoint id to pos, so it now contains 3
            # items: x, y, waypoint id
            pos.append(nearest)

        rospy.loginfo(self.stop_lines)

        ''' 
        Code below is to see how yaw can be computed from
        x,y points.  Conclusion: yaw in the input file at point i 
        appears to be computed by finding the angle of the line 
        that goes from point i-1 to point i+1.
        '''

            
        '''
        rospy.loginfo("yaw:")
        # rg = range(0,10)
        # rg.extend(range(len(self.wps)-10, len(self.wps)))
        # rg = range(1823,1833)
        rg = range(0,len(self.wps))
        rp = 0.
        rn = 0.
        rb = 0.
        emin = 0.
        emax = 0.
        for i in rg:
            wp = self.wps[i]
            q = wp.pose.pose.orientation
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            x = wp.pose.pose.position.x
            y = wp.pose.pose.position.y
            cap = 0.
            can = 0.
            cab = 0.
            if i > 0 and i < len(self.wps)-1:
                wpp = self.wps[i-1]
                px = wpp.pose.pose.position.x
                py = wpp.pose.pose.position.y
                wpn = self.wps[i+1]
                nx = wpn.pose.pose.position.x
                ny = wpn.pose.pose.position.y
                cap = math.atan2(y-py, x-px) - yaw
                can = math.atan2(ny-y, nx-x) - yaw
                cab = math.atan2(ny-py, nx-px) - yaw
                rp += cap*cap
                rn += can*can
                rb += cab*cab
                err = math.degrees(cab)
                if err > 180:
                    err -= 360
                if err < -180:
                    err += 360
                emin = min(err, emin)
                emax = max(err, emax)
            # rospy.loginfo("%d %f %f %f %f  %.3f %.2f", i, yaw, cap, can, cab, x, y)
        rospy.loginfo("err min max %f %f", math.radians(emin), math.radians(emax))
        '''

        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # print("tcb", msg)
        # If next_tl is positive, then next_tl has the waypoint
        # of the next red light.  If next_tl is negative, then
        # abs(next_tl) has the waypoint of the next light, and the
        # negative sign signals that the next light is NOT red.
        
        ## Juan's Comment:
        ## had to put this threshold to avoid false positives

        if msg.data < 0: # no red light
            self.red_tl = False
        else:
            self.red_tl = True
        
        if self.red_tl_prev == self.red_tl == False:
            self.tl_count += 1
            if self.tl_count < 3:
                self.red_tl = True
            else:
                self.red_tl = False

        else:
            self.red_tl_prev = self.red_tl
            self.red_tl = True
            self.tl_count = 0



        rospy.logwarn('[traffic_cb] traffict ligth: %s', self.red_tl)
        next_tl = msg.data
        sgn = 1
        if next_tl < 0:
            sgn = -1
            next_tl *= -1
        dist = -1.0
        if self.next_pt >= 0 and next_tl >= 0:
            sz = len(self.wps)
            dist = self.wp_ss[next_tl] - self.wp_ss[self.next_pt]
            if dist < 0:
                if dist < -300:
                    dist += self.wp_ss[sz]
                else:
                    dist = 0

        # Output: distance to next light.  If output is positive,
        # it means that the next light is red.  If distance is negative,
        # it means that the next light is NOT red, and abs(distance)
        # gives the distance to this non-red light.
        self.tl_distance_pub.publish(sgn*dist)
        #self.red_tl = True if sgn*dist > 0 else False
        # print("ds", dist)
       


    def obstacle_cb(self, msg):
        # This is never called
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

'''
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
'''


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
