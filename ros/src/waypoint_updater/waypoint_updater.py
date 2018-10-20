#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import math
import copy
import yaml
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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL=.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.base_lane=None
        self.base_lane_left=None
        self.base_lane_right=None
        self.pose=None
        self.stopline_wp_idx=-1
        self.waypoints_2d=None
        self.waypoint_tree=None
        self.lasttraffic=0
        self.lane=1#0,1,2
        self.lanetime=0
        self.prev_pose=None
        self.stopobstacle_idx=-1
        self.obstacles=np.array([])

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_line_positions = self.config['stop_line_positions']
        
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/vehicle/obstacle_points', PointCloud2, self.obstacle_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.loop()
    def loop(self):
        rate=rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()
    def get_closest_waypoint_idx(self):
        x=self.pose.pose.position.x
        y=self.pose.pose.position.y
        if self.waypoint_tree!=None:
            closest_idx=self.waypoint_tree.query([x,y],1)[1]
            #check if closest is ahead or behind vehicle
            closest_coord=self.waypoints_2d[closest_idx]
            prev_coord=self.waypoints_2d[closest_idx-1]
            #equation for hyperplane through closest_coords
            cl_vect=np.array(closest_coord)
            pre_vect=np.array(prev_coord)
            pos_vect=np.array([x,y])
            val=np.dot(cl_vect-pre_vect,pos_vect-cl_vect)
            if val>0:
                closest_idx=(closest_idx+1) % len(self.waypoints_2d)
            return closest_idx
        return 0
    def get_closest_waypoint_idx1(self,p):
        x=p[0]
        y=p[1]
        if self.waypoint_tree!=None:
            closest_idx=self.waypoint_tree.query([x,y],1)[1]
            #check if closest is ahead or behind vehicle
            closest_coord=self.waypoints_2d[closest_idx]
            prev_coord=self.waypoints_2d[closest_idx-1]
            #equation for hyperplane through closest_coords
            cl_vect=np.array(closest_coord)
            pre_vect=np.array(prev_coord)
            pos_vect=np.array([x,y])
            val=np.dot(cl_vect-pre_vect,pos_vect-cl_vect)
            if val>0:
                closest_idx=(closest_idx+1) % len(self.waypoints_2d)
            return closest_idx
        return 0
    #select the best lane,if the current lane has obstacles,than change to other lane
    def choose_best_lane(self,obstacles,lane,s,snum):
        close_s=[]
        item=[]
        item.append(100000)
        item.append(0)
        item1=[]
        item1.append(item)
        item1.append(item)
        close_s.append(item1)
        item=[]
        item.append(100000)
        item.append(0)
        item1=[]
        item1.append(item)
        item1.append(item)
        close_s.append(item1)
        item=[]
        item.append(100000)
        item.append(0)
        item1=[]
        item1.append(item)
        item1.append(item)
        close_s.append(item1)
        #calculate the most nearest car before or after the ego car in each lane,get their distance and velocity
        for i in range(len(obstacles)):
            d=obstacles[i][1]
            if(d>0)and(d<12):
                if d>8:
                    check_lane=2
                else:
                    if d<4:
                        check_lane=0
                    else:
                        check_lane=1
                check_car_s=obstacles[i][0]
                pre_dis=(check_car_s-s+snum)%snum
                after_dis=(s-check_car_s+snum)%snum
                if (close_s[check_lane][0][0]>pre_dis):
                    close_s[check_lane][0][0]=pre_dis
                    close_s[check_lane][0][1]=0
                if (close_s[check_lane][1][0]>after_dis):
                    close_s[check_lane][1][0]=after_dis
                    close_s[check_lane][1][1]=0
        #cout<<close_s[0][1][0]<<","<<close_s[1][1][0]<<","<<close_s[2][1][0]<<endl
        #calculate the cost of each lane
        #print(close_s)
        costs=[0,0,0]
        for j in range(3):
            if (close_s[j][0][0]<=50):
            #if the distance of the car that is before the ego car in that lane is less then 30 meters,don't change
                costs[j]=10000
            else:
                if(j!=lane):
                    #if the distance of the car that is after the ego car in that lane is less then 15 meters,don't change
                    if(close_s[j][1][0]<15):
                        costs[j]=10000
                if (costs[j]==0):
                    tems=close_s[j][0][0]
                    cost1=1 - math.exp(-1*(1/(tems)))
                    costs[j] = 1000*cost1
            
        
        #decrease the cost of current lane by 1,in order to hold the current lane if it is equal to others.
        costs[lane]-=1
        #cout<<costs[0]<<","<<costs[1]<<","<<costs[2]<<endl
        min_cost=costs[0]
        min_lane=0
        #select the minimum cost lane
        for j in range(3):
            if (min_cost>costs[j]):
                min_cost=costs[j]
                min_lane=j
        #if change lane from 0 to 2 or from 2 to 0,than look at the cost of lane 1.If the cost of lane 1 is 10000,don't change lane.Otherwise,change to lane 1 first. 
        if(abs(min_lane-lane)==2):
            if (costs[1]<10000):
                return 1
            else:
                return lane
        return min_lane
    #whether the car is too close to the obstacles,if current lane has obstacles
    def istooclose(self,closest_idx):
        size=len(self.obstacles)
        too_close=False
        self.stopobstacle_idx=-1
        closedis=10000
        for i in range(size):
            d=self.obstacles[i][1]
            s=self.obstacles[i][0]
            
            if(d<(2+4*self.lane+2)+1 and d>(2+4*self.lane-2)-1):#the vehicle which is changing to this line is also considered as in this lane
                if((s>closest_idx)and((s-closest_idx)<30)):
                    too_close = True
                    if closedis>s-closest_idx:
                        closedis=s-closest_idx
        if closedis==10000:
            self.stopobstacle_idx=-1
        else:
            self.stopobstacle_idx=closedis
        return too_close
    def publish_waypoints(self):
        if self.waypoint_tree:
                final_lane=self.generate_lane()
                self.final_waypoints_pub.publish(final_lane)
    #convert the  s,d coordinates to x,y coordinates
    def getXY(self,s, d,waypoints):
        d=d-6
        wlen=len(waypoints)
        ret=copy.deepcopy(waypoints[s])
        ps=(s-1+wlen)%wlen
        x2=waypoints[s].pose.pose.position.x
        y2=waypoints[s].pose.pose.position.y
        x1=waypoints[ps].pose.pose.position.x
        y1=waypoints[ps].pose.pose.position.y
        alf=math.atan2(y2-y1,x2-x1)
        #print(alf)
        #print(str(ps)+','+str(x1)+','+str(y1)+'=='+str(s)+','+str(x2)+','+str(y2))
        if (y2-y1>=0) and (x2-x1>=0):
            alf=math.pi/2-alf
            y0=y2-d*math.sin(alf)
            x0=x2+d*math.cos(alf)
        if (y2-y1>=0) and (x2-x1<0):
            alf=math.pi/2+alf
            y0=y2-d*math.sin(alf)
            x0=x2-d*math.cos(alf)
        if (y2-y1<0) and (x2-x1>=0):
            alf=math.pi/2+alf
            y0=y2-d*math.sin(alf)
            x0=x2-d*math.cos(alf)
        if (y2-y1<0) and (x2-x1<0):
            alf=math.pi/2-alf
            y0=y2-d*math.sin(alf)
            x0=x2+d*math.cos(alf)
        #print(str(ps)+','+str(x1)+','+str(y1)+'=='+str(s)+','+str(x2)+','+str(y2)+'=='+str(x0)+','+str(y0)+'=='+str(alf))
        ret.pose.pose.position.x=x0
        ret.pose.pose.position.y=y0
        return ret
    #calculate the distance from current car position to the nearest front stop_line which is read from config file 
    def distostopline(self,closest_idx):
        ret=10000
        for i in range(len(self.stop_line_positions)):
            stopline_idx=self.get_closest_waypoint_idx1(self.stop_line_positions[i])
            dis=stopline_idx-closest_idx
            if  (dis>0) and (ret>dis):
                ret=dis
        return ret
        
    def generate_lane(self):
        lane=Lane()
        closest_idx=self.get_closest_waypoint_idx()
        stopdis=self.distostopline(closest_idx)
        #select new lane
        if (stopdis>50):
            if len(self.obstacles)>0:
                self.lane=self.choose_best_lane(self.obstacles,self.lane,closest_idx,len(self.base_lane.waypoints))
        #print('newlane:'+str(self.lane))
        farthest_idx=closest_idx+LOOKAHEAD_WPS
        if self.lane==0:
            base_waypoints=copy.deepcopy(self.base_lane_left.waypoints[closest_idx:farthest_idx])
        if self.lane==1:
            base_waypoints=copy.deepcopy(self.base_lane.waypoints[closest_idx:farthest_idx])
        if self.lane==2:
            base_waypoints=copy.deepcopy(self.base_lane_right.waypoints[closest_idx:farthest_idx])
        if len(base_waypoints)>10:
            x1=self.pose.pose.position.x
            y1=self.pose.pose.position.y
            x2=base_waypoints[0].pose.pose.position.x
            y2=base_waypoints[0].pose.pose.position.y
            x3=base_waypoints[1].pose.pose.position.x
            y3=base_waypoints[1].pose.pose.position.y
            #if change lane,than caculate the first 10 points of the path
            if self.distance2line([x1,y1],[x2,y2],[x3,y3])>1:
                x2=base_waypoints[10].pose.pose.position.x
                y2=base_waypoints[10].pose.pose.position.y
        
                for i in range(10):    
                    base_waypoints[i].pose.pose.position.y=(y2-y1)*(i+2)/10+y1
                    base_waypoints[i].pose.pose.position.x=(x2-x1)*(i+2)/10+x1
                
        #judge if too close
        self.stopobstacle_idx=-1
        if len(self.obstacles)>0:
            self.istooclose(closest_idx)
        if (self.stopobstacle_idx!=-1):
            print('obs_point:'+str(self.stopobstacle_idx))
            if (self.stopline_wp_idx!=-1 and (self.stopline_wp_idx<farthest_idx)):
                stop_point=min(self.stopobstacle_idx,self.stopline_wp_idx)
            else:
                stop_point=self.stopobstacle_idx
        else:
            if (self.stopline_wp_idx!=-1 and (self.stopline_wp_idx<farthest_idx)):
                stop_point=self.stopline_wp_idx
            else:
                stop_point=-1
        
        #print('stop:'+str(self.stopline_wp_idx))    
        if stop_point!=-1:        
            lane.waypoints=self.decelerate_waypoints(base_waypoints,closest_idx,stop_point)
        else:
            lane.waypoints=base_waypoints
        
        return lane
    def decelerate_waypoints(self,waypoints,closest_idx,stop_point):
        temp=[]
        #rospy.loginfo('%s %s %s',closest_idx,self.stopline_wp_idx,len(waypoints))
        for i,wp in enumerate(waypoints):
            p=Waypoint()
            p.pose=wp.pose
            stop_idx=max(stop_point-closest_idx-2,0)
            dist=self.distance(waypoints,i,stop_idx)
            vel=math.sqrt(2*MAX_DECEL*dist)
            if vel<1.:
                vel=0.
            p.twist.twist.linear.x=min(vel,wp.twist.twist.linear.x)
            temp.append(p)
        return temp
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose=msg
        #rospy.loginfo('self_pos:%s',self.pose.pose.position.x)
        

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        print('waypoints_cb')
        self.base_lane=waypoints
        self.base_lane_left=copy.deepcopy(waypoints)
        self.base_lane_righ=copy.deepcopy(waypoints)
        for i in range(len(self.base_lane.waypoints)):
            self.base_lane_left.waypoints[i]=self.getXY(i,2,self.base_lane.waypoints)
            self.base_lane_righ.waypoints[i]=self.getXY(i,10,self.base_lane.waypoints)
            if not self.waypoints_2d:
                self.waypoints_2d=[[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
                self.waypoint_tree=KDTree(self.waypoints_2d)

        
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        if self.lasttraffic!=msg.data:
            #rospy.loginfo('traffic:%s',msg.data)
            self.lasttraffic=msg.data
        self.stopline_wp_idx=msg.data
    def distance2line(self,p0,p1,p2):
        x0=p0[0]
        y0=p0[1]
        x1=p1[0]
        y1=p1[1]
        x2=p2[0]
        y2=p2[1]
        d=math.sqrt((y2-y1)*(y2-y1)+(x1-x2)*(x1-x2))
        return (x0*(y2-y1)+y0*(x1-x2)+y1*x2-x1*y2)/d
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        obstacle_list=[]
        if self.waypoint_tree:
            for p in pc2.read_points(msg,skip_nans=True,field_names=('x','y','z')):
                closest_idx=self.waypoint_tree.query([p[0],p[1]],1)[1]
                prev_idx=(closest_idx-1)% len(self.waypoints_2d)
                closest_coord=self.waypoints_2d[closest_idx]
                prev_coord=self.waypoints_2d[closest_idx-1]
                    #equation for hyperplane through closest_coords
                cl_vect=np.array(closest_coord)
                pre_vect=np.array(prev_coord)
                pos_vect=np.array([p[0],p[1]])
                val=np.dot(cl_vect-pre_vect,pos_vect-cl_vect)
                if val>0:
                    prev_idx=closest_idx
                    closest_idx=(closest_idx+1) % len(self.waypoints_2d)
                distance=self.distance2line(pos_vect,self.waypoints_2d[prev_idx],self.waypoints_2d[closest_idx])
                obstacle_list.append([prev_idx,distance+6])
            self.obstacles=np.array(obstacle_list)

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

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
