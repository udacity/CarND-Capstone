import copy
import math

class WaypointTracker(object):
    def __init__(self):
        self.base_waypoints = None
        self.pose = None

        self.last_closest_front_waypoint_index = 0

        # this is a super_class, so it will not start loop nor spin()
        # it expects the subclass will implement the appropriate

    def base_waypoints_process(self, msg):
        # DONE: Implement
        waypoints = msg.waypoints
        if self.base_waypoints is None:
            # unsubscribe to the waypoint messages, no longer needed
            self.base_waypoints_sub.unregister()
            self.subscriber_waypoints = None
    
            self.base_waypoints_num = len(waypoints)
    
            # process the waypoints here
            self.dist_to_here_from_start = []
            self.base_waypoints = []
            dist = 0
            dist_so_far = 0
            self.shortest_dist_to_next_waypoint = 0
            for i in range(self.base_waypoints_num):
                dist_so_far += dist
                self.dist_to_here_from_start.append(dist_so_far)
    
                # do a deep copy of the data, to keep the data from lose
                # just to be safe, simply do shallow copy seems still working
                # by self.base_waypoints = waypoints
                self.base_waypoints.append(copy.deepcopy(waypoints[i]))
                # distance to the next waypoint
                if (i < self.base_waypoints_num-1):
                    dist = (
                        self.distance(waypoints,  # the (i+1)_th element has not been copied yet
                                             i, (i+1) % self.base_waypoints_num))
                # end of if (i < self.base_waypoints_num-1)
                if (dist < self.shortest_dist_to_next_waypoint):
                    self.shortest_dist_to_next_waypoint = dist
                # end of if (dist < self.shortest_dist_to_next_waypoint)
            # end of for i in range(self.base_waypoints_num - 1)
        # end of if self.base_waypoints is None
    def current_pose_cb(self, msg):
        # WORKING: Implement
        #
        if self.pose is None:       # ready to process message
            self.pose = msg
        # end of if self.pose is None
        # otherwise, the current message is being processed, rejected the coming message and expect to receive more updated next one.
    
    def distance(self, waypoints, wp1, wp2):
            dist = 0
            dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
            for i in range(wp1, wp2+1):
                dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
                wp1 = i
            return dist
