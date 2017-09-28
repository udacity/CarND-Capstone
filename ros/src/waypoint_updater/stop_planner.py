import numpy as np
import math
from scipy.misc import derivative



class StopPlanner(object):
    def __init__(self):
        self.map_s = None
        #self.getMap_s(waypoints)
        


    def ClosestWaypoint(self, x, y, waypoints):
        closestLen = 99999
        closesWaypoint = 0
        for i in range(len(waypoints)):
            wpx = waypoints[i].pose.pose.position.x
            wpy = waypoints[i].pose.pose.position.y
            dist = self.euclidean_distance(x, y, wpx, wpy)
            if dist < closestLen:
                closestLen = dist
                closesWaypoint = i
        return closesWaypoint

    def NextWaypoint(self, x, y, theta, waypoints):
        closestWaypoint = self.ClosestWaypoint(x, y, waypoints)
        wpx = waypoints[closestWaypoint].pose.pose.position.x
        wpy = waypoints[closestWaypoint].pose.pose.position.y
        heading = math.atan2(wpy - y, wpx - x)
        angle = abs(theta - heading)
        if angle > (np.pi/4.):
            closestWaypoint += 1
        return closestWaypoint



    ## calculate euclidean distance between two points
    #
    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    ## calculate map of "s" used to convert to frenet coordinates
    ##
    def getMap_s(self, waypoints):
        map_s = [0.]
        s = 0
        for i in range(len(waypoints)-1):
            s += self.euclidean_distance(waypoints[i].pose.pose.position.x,
                                    waypoints[i].pose.pose.position.y,
                                    waypoints[i+1].pose.pose.position.x,
                                    waypoints[i+1].pose.pose.position.y)
            map_s.append(s)
        return map_s

    ## calculate distance between two waypoints
    #
    def distance(self, waypoints, wp1, wp2):
            dist = 0
            dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
            for i in range(wp1, wp2+1):
                dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
                wp1 = i
            return dist


    ## Transform from Frenet s,d coordinates to Cartesian x,y
    def getXY(self, s, d, maps_s, waypoints):
        prev_wp = -1
        while (s > maps_s[prev_wp+1]) and (prev_wp < len(maps_s) - 1): 
            prev_wp += 1
        wp2 = (prev_wp+1)%len(waypoints)
        heading = math.atan2((waypoints[wp2].pose.pose.position.y-waypoints[prev_wp].pose.pose.position.y),
                                (waypoints[wp2].pose.pose.position.x-waypoints[prev_wp].pose.pose.position.x))
        ## the x,y,s along the segment
        seg_s = (s-maps_s[prev_wp])
        seg_x = waypoints[prev_wp].pose.pose.position.x+seg_s*math.cos(heading)
        seg_y = waypoints[prev_wp].pose.pose.position.y+seg_s*math.sin(heading)
        perp_heading = heading-np.pi/2
        x = seg_x + d*math.cos(perp_heading)
        y = seg_y + d*math.sin(perp_heading)
        return [x,y]


    ## Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    def getFrenet(self, x,  y,  theta, waypoints):
        next_wp = self.NextWaypoint(x,y, theta, waypoints)
        prev_wp = next_wp-1
        if(next_wp == 0):
            prev_wp = len(waypoints) - 1
        n_x = waypoints[next_wp].pose.pose.position.x-waypoints[prev_wp].pose.pose.position.x
        n_y = waypoints[next_wp].pose.pose.position.y-waypoints[prev_wp].pose.pose.position.y
        x_x = x - waypoints[prev_wp].pose.pose.position.x
        x_y = y - waypoints[prev_wp].pose.pose.position.y
        ## find the projection of x onto n
        proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
        proj_x = proj_norm*n_x
        proj_y = proj_norm*n_y
        frenet_d = self.euclidean_distance(x_x,x_y,proj_x,proj_y)
        ##see if d value is positive or negative by comparing it to a center point
        center_x = 1000-waypoints[prev_wp].pose.pose.position.x
        center_y = 2000-waypoints[prev_wp].pose.pose.position.y
        centerToPos = self.euclidean_distance(center_x,center_y,x_x,x_y)
        centerToRef = self.euclidean_distance(center_x,center_y,proj_x,proj_y)
        if(centerToPos <= centerToRef):
            frenet_d *= -1
        ## calculate s value
        frenet_s = 0
        for i in range(prev_wp):
            frenet_s += self.euclidean_distance(waypoints[i].pose.pose.position.x,waypoints[i].pose.pose.position.y,
                                            waypoints[i+1].pose.pose.position.x, waypoints[i+1].pose.pose.position.y)
        frenet_s += self.euclidean_distance(0,0,proj_x,proj_y)
        return [frenet_s,frenet_d]


    def JMT(self, start, end, T):
        """
        Calculates Jerk Minimizing Trajectory for start, end and T.
        """
        a_0, a_1, a_2 = start[0], start[1], start[2] / 2.0
        c_0 = a_0 + a_1 * T + a_2 * T**2
        c_1 = a_1 + 2* a_2 * T
        c_2 = 2 * a_2
        
        A = np.array([
                [  T**3,   T**4,    T**5],
                [3*T**2, 4*T**3,  5*T**4],
                [6*T,   12*T**2, 20*T**3],
            ])
        B = np.array([
                end[0] - c_0,
                end[1] - c_1,
                end[2] - c_2
            ])
        a_3_4_5 = np.linalg.solve(A,B)
        alphas = np.concatenate([np.array([a_0, a_1, a_2]), a_3_4_5])
        # reverse coefficients to match order used by np.poly1d
        return alphas[::-1]




'''

def JMT(start, end, T):
    """
    Calculates Jerk Minimizing Trajectory for start, end and T.
    """
    a_0, a_1, a_2 = start[0], start[1], start[2] / 2.0
    c_0 = a_0 + a_1 * T + a_2 * T**2
    c_1 = a_1 + 2* a_2 * T
    c_2 = 2 * a_2
    
    A = np.array([
            [  T**3,   T**4,    T**5],
            [3*T**2, 4*T**3,  5*T**4],
            [6*T,   12*T**2, 20*T**3],
        ])
    B = np.array([
            end[0] - c_0,
            end[1] - c_1,
            end[2] - c_2
        ])
    a_3_4_5 = np.linalg.solve(A,B)
    alphas = np.concatenate([np.array([a_0, a_1, a_2]), a_3_4_5])
    return alphas






def ClosestWaypoint(x, y, waypoints):
    closestLen = 99999
    closesWaypoint = 0
    for i in range(len(waypoints)):
        wpx = waypoints[i][0]
        wpy = waypoints[i][1]
        dist = euclidean_distance(x, y, wpx, wpy)
        if dist < closestLen:
            closestLen = dist
            closesWaypoint = i
    return closesWaypoint

def NextWaypoint(x, y, theta, waypoints):
    closestWaypoint = ClosestWaypoint(x, y, waypoints)
    wpx = waypoints[closestWaypoint][0]
    wpy = waypoints[closestWaypoint][1]
    heading = math.atan2(wpy - y, wpx - x)
    angle = abs(theta - heading)
    if angle > (np.pi/4.):
        closestWaypoint += 1
    return closestWaypoint
'''


if __name__ == "__main__":
    pass