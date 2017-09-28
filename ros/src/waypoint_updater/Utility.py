import tf
import math
import rospy
from scipy import interpolate
import pycubicspline as spline
import numpy as np

def getSplineCoeffs(x_points,y_points):
    nx = np.asarray(x_points)
    ny = np.asarray(y_points)
    return spline.Spline2D(nx,ny)
    #return interpolate.splrep(x_points,y_points, k=3)
    
def fitX(x,tck):
    nx = np.asarray(x)
    return tck.calc_position(nx)
    #return interpolate.splev(x,tck)

def getHeading(quaternion):
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return yaw
    
def getQuaternion(roll,pitch,yaw):
    return tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    
def generateMapS(map_x,map_y):
    map_s = []
    
    if True:
        map_s.append(0) # starts at 0
        for i in range(1,len(map_x)):
            s = distance(map_x[i-1],map_y[i-1],map_x[i],map_y[i])
            # Add to the previous s
            s += map_s[-1]
            # Append to the list
            map_s.append(s)
    else:
        for x,y in zip(map_x,map_y):
            s = convertToFrenet(x,y,0,map_x,map_y)
            if len(map_s) is not 0:
                s += map_s[-1]
            map_s.append(s)
        
    return map_s

def distance(x1,y1,x2,y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

def closestWaypoint(x, y, map_x, map_y, last_idx=None):
    '''
    Get the closes waypoint to the vehicle
    '''
    index = 0
    minD = 100000
    count = 0
    idx = 0

    for mx, my in zip(map_x, map_y):
        d = distance(x,y,mx,my)
        if d < minD:
            minD = d
            index = count
        count += 1
    return index

def nextWaypoint(x, y, theta, map_x, map_y, last_idx=None):
    '''
    Get the next waypoint in front of the vehicle
    '''
    idx = closestWaypoint(x,y,map_x,map_y, last_idx=last_idx)
    mx = map_x[idx]
    my = map_y[idx]
    heading = math.atan2((my-y),(mx-x))
    angle = abs(theta - heading)
    if angle > math.pi / 4:
        idx += 1
    idx %= len(map_x)
    return idx

def convertToFrenet(x, y, theta, map_x, map_y):
    '''
    Convert from x / y coordinates to Frenet coordinates
    '''
    
    # Get next waypoint index and previous one. Handle wrap around
    next_idx = nextWaypoint(x,y,theta,map_x,map_y)
    prev_idx = next_idx - 1
    if next_idx == 0:
        prev_idx = len(map_x) - 1
    
    # Find project of x onto n
    nx = map_x[next_idx] - map_x[prev_idx]
    ny = map_y[next_idx] - map_y[prev_idx]
    xx = x -  map_x[prev_idx]
    yy = y -  map_y[prev_idx]
    
    proj_norm = (xx*nx + yy*ny)/(nx*nx + ny*ny)
    proj_x = proj_norm * nx
    proj_y = proj_norm * ny
    
    frenet_d = distance(xx,yy,proj_x,proj_y)
    
    # See if d should be positive or negative
    c_x = 1000-map_x[prev_idx]
    c_y = 2000-map_y[prev_idx]
    c_pos = distance(c_x,c_y,xx,yy)
    c_ref = distance(c_x,c_y,proj_x,proj_y)
    
    if(c_pos <= c_ref):
        frenet_d *= -1
    
    frenet_s = 0
    for i in range(0,prev_idx):
        frenet_s += distance(map_x[i],map_y[i],map_x[i+1],map_y[i+1])
    
    frenet_s += distance(0,0,proj_x,proj_y)
    
    # This final distance is weird... return it and offset X
    return frenet_s,frenet_d

def convertFromFrenet(s, d, map_s, map_x, map_y):
    prev_idx = -1
    while(s > map_s[prev_idx+1] and prev_idx < len(map_s)-2):
        prev_idx += 1;
    wp = (prev_idx+1) % len(map_s)
    
    heading = math.atan2(map_y[wp] - map_y[prev_idx],map_x[wp] - map_x[prev_idx])
    seg_s = s - map_s[prev_idx]
    
    seg_x = map_x[prev_idx] + seg_s * math.cos(heading)
    seg_y = map_y[prev_idx] + seg_s * math.sin(heading)
    
    p_heading = heading - math.pi/2
    
    x = seg_x + d*math.cos(p_heading)
    y = seg_y + d*math.sin(p_heading)
    
    return x, y
    
