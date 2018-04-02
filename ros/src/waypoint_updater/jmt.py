#!/usr/bin/env python

import rospy
import math
import numpy as np


class JMT(object):
    def __init__(self, start, end, T):

        """
        Calculates Jerk Minimizing Trajectory for start, end and T.
        start and end include
        [displacement, velocity, acceleration]
        """
        self.start = start
        self.end = end
        self.final_displacement = end[0]
        self.T = T

        a_0, a_1, a_2 = start[0], start[1], start[2] / 2.0
        c_0 = a_0 + a_1 * T + a_2 * T**2
        c_1 = a_1 + 2 * a_2 * T
        c_2 = 2 * a_2

        A = np.array([
                     [T**3,   T**4,    T**5],
                     [3*T**2, 4*T**3,  5*T**4],
                     [6*T,   12*T**2, 20*T**3],
                     ])

        B = np.array([
                     end[0] - c_0,
                     end[1] - c_1,
                     end[2] - c_2
                     ])
        a_3_4_5 = np.linalg.solve(A, B)
        self.coeffs = np.concatenate([np.array([a_0, a_1, a_2]), a_3_4_5])

    # def JMTD_at(self, displacement, coeffs, t0, tmax, deq_wpt_ptr):
    def JMTD_at(self, displacement, t0, tmax):
        # find JMT descriptors at displacement
        s_last = 0.0
        t_found = False
        t_inc = 0.01
        iterations = 0

        for t_cnt in range(int((tmax-t0)/t_inc) + 100):
            iterations += 1
            t = t0 + t_cnt * t_inc
            s = self.get_s_at(t)
            if s > displacement:
                t = t - (1 - (s - displacement) / (s - s_last)) * t_inc
                t_found = True
                break
            # end if
            s_last = s
        # end for
        if t_found is False:
            rospy.loginfo("waypoint_updater:JMTD_at Ran out of bounds without "
                          "finding target displacement")
            return None

        s = self.get_s_at(t)
        delta_s = (displacement - s)
        if delta_s > 0.0:
            searchdir = 1.0
        else:
            searchdir = -1.0

        t_smallinc = searchdir * 0.005
        while delta_s * searchdir > 0.0:
            iterations += 1
            t += t_smallinc
            delta_s = displacement - self.get_s_at(t)

        rospy.logdebug("delta_s = {}, t= {}, iterations = {}".format(
            delta_s, t, iterations))
        if delta_s > 0.1:
            rospy.loginfo("waypoint_updater:JMTD_at need to refine algo,"
                          " delta_s is {}".format(delta_s))

        details = JMTDetails(self.get_s_at(t), self.get_v_at(t),
                             self.get_a_at(t), self.get_j_at(t), t)

        rospy.logdebug("waypoint_updater:JMTD_at displacement {} found "
                       "s,v,a,j,t = {}".format(displacement, details))

        return details

    def get_s_at(self, t):
        return self.coeffs[0] + self.coeffs[1] * t + self.coeffs[2] * t**2 +\
            self.coeffs[3] * t**3 + self.coeffs[4] * t**4 + self.coeffs[5] *\
            t**5

    def get_v_at(self, t):
        return self.coeffs[1] + 2.0 * self.coeffs[2]*t + 3.0 *\
            self.coeffs[3] * t**2 + 4.0 * self.coeffs[4] *\
            t**3 + 5.0 * self.coeffs[5] * t**4

    def get_a_at(self, t):
        return 2.0 * self.coeffs[2] + 6.0 * self.coeffs[3] * t + 12.0 *\
            self.coeffs[4] * t**2 + 20.0 * self.coeffs[5] * t**3

    def get_j_at(self, t):
        return 6.0 * self.coeffs[3] + 24.0 * self.coeffs[4] * t + 60.0 *\
            self.coeffs[5] * t**2


class JMTDetails(object):
    def __init__(self, S, V, A, J, t):
        self.S = S
        self.V = V
        self.A = A
        self.J = J
        self.time = t

    def set_VAJt(self, V, A, J, time):
        self.V = V
        self.A = A
        self.J = J
        self.time = time

    def __repr__(self):
        return "%3.3f, %6.3f, %2.4f, %2.4f, %2.4f" % (self.time, self.S,
                                                      self.V, self.A,
                                                      self.J)


class JMTD_waypoint(object):
    def __init__(self, waypoint, ptr_id, s):
        # self.position = pose(xval, yval, zval)
        self.waypoint = waypoint
        self.ptr_id = ptr_id
        self.max_v = self.get_v()  # max_v is read only
        self.set_v(0.0)
        self.JMTD = JMTDetails(s, 0.0, 0.0, 0.0, 0.0)
        self.state = None
        self.JMT_ptr = -1  # points to JMT object

    def set_v(self, v):
        # put a check for max_v here
        if v > self.max_v:
            rospy.logwarn("trying to set velocity to {:2.2f}, "
                          "but limited to {:2.2f} at ptr = {}"\
                          .format(v, self.max_v, self.ptr_id))
        self.waypoint.twist.twist.linear.x = min(v, self.max_v)
        # self.JMTD.V = v

    def get_v(self):
        return self.waypoint.twist.twist.linear.x

    def get_position(self):
        return self.waypoint.pose.pose.position

    def get_x(self):
        return self.waypoint.pose.pose.position.x

    def get_y(self):
        return self.waypoint.pose.pose.position.y

    def get_z(self):
        return self.waypoint.pose.pose.position.z

    def get_s(self):
        return self.JMTD.S

    def get_a(self):
        return self.JMTD.A

    def get_j(self):
        return self.JMTD.J

    def get_maxV(self):
        return self.max_v
