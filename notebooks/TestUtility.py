import sys
import sys
sys.path.insert(0, '/home/j3/dev/projects/personal/Udacity/CarND-Capstone/ros/src/waypoint_updater')
import Utility as util
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
import matplotlib.rcsetup as rcsetup
import numpy as np



import pandas as pd
names =['x','y','special','heading']
sim_map = pd.read_csv('../data/wp_yaw_const.csv', names=names)

def test_location(x,y, mx, my):
    idx = util.nextWaypoint(x,y,0.0872013443653,mx,my)
    distance = util.distance(x,y,mx[idx],my[idx])
    #if(mx[idx] < idx):
    print(x,y,mx[idx],my[idx], distance)
    
def test_idx(idx,mh,mx,my):
    nidx = util.nextWaypoint(mx[idx], my[idx]+0.1,mh[idx],mx,my)
    if nidx <= idx:
        print(nidx, idx)
    #if(mx[nidx]+.01 < my[idx] or my[nidx]+.01 < my[idx] ):
    #    print('Old, New (',idx,nidx,') (',mx[idx],my[idx], ') (', mx[nidx], my[nidx], ')')

    
test_location(1261.439,1170.68,sim_map['x'],sim_map['y'])
test_location(1131.461,1183.291,sim_map['x'],sim_map['y'])
test_location(1477.888,1212.58,sim_map['x'],sim_map['y'])


if False:
    for i in range(0,1000):
            test_idx(i,sim_map['heading'],sim_map['x'],sim_map['y'])

if False:     
    print("Testing Spline Functionality")  

    x = [1128.11, 1136.03, 1148.37, 1155.42]
    y = [1183.12, 1183.86, 1184.83, 1185.29]
    print('Spline fit between:',x,y)
    sp = util.getSplineCoeffs(x,y)
    toFit = range(1129,1155)

    rx, ry = [], []

    s = np.arange(0, sp.s[-1], 0.1)
    for i in s:
        ix,iy = util.fitX(i,sp)
        rx.append(ix)
        ry.append(iy)
        
    if True:
        plt.figure()
        plt.scatter(x,y)
        plt.plot(rx,ry)
        plt.show()
        
        
idx = util.nextWaypoint(1129,1155,0,sim_map['x'],sim_map['y'])

print('Next IDX',idx)

if True:
    print('******************************')
    print('******Map Utility Testing')
    print('******************************')
    for theta in range(-40,40):
        idx = util.nextWaypoint(1252.197,2964.488, theta*.1, sim_map['x'],sim_map['y'])
        print(theta, idx,util.distance(1252.197,2964.488,sim_map['x'][idx],sim_map['y'][idx]))
    exit()
    
    
if False:
    print('******************************')
    print('******Frenet Testing')
    print('******************************')
    map_s = util.generateMapS(sim_map['x'],sim_map['y'])
    index = 0
    for i in range(len(sim_map['x'])-5,len(sim_map['x'])):
    #for xp, yp in zip(sim_map['x'],sim_map['y']):
        xp = sim_map['x'][i]
        yp = sim_map['y'][i]
        s,d = util.convertToFrenet(xp,yp,sim_map['heading'][index],sim_map['x'],sim_map['y'])
        xx, yy = util.convertFromFrenet(s, d, map_s, sim_map['x'], sim_map['y'])
        if abs(xx-xp) > 0.1 or abs(yy-yp) > 0.1:
            print(index, xp,xx,yp,yy)
        index += 1
    

print('******************************')
print('******Spline Testing')
print('******************************')


def testGitSpline(index,start_x=None,start_y=None, theta=None):
    
    map_s = util.generateMapS(sim_map['x'],sim_map['y'])
    if not start_x:
        start_x = sim_map['x'][index] # - 3.24
    if not start_y:
        start_y = sim_map['y'][index] # + 1.6
    if theta:
        print(util.nextWaypoint(start_x,start_y,theta,sim_map['x'],sim_map['y']))
    
    # Convert the start position to frenet coordinates
    s,d = util.convertToFrenet(start_x,start_y,0,sim_map['x'],sim_map['y'])
    #print'X,Y', start_x, start_y, ' >> S,D', s, d
    #print'S,D', s, d, ' >> X,Y', util.convertFromFrenet(s, d, map_s, sim_map['x'], sim_map['y'])

    # Create and the array of points to fit the spline to
    px = [start_x]
    py = [start_y]

    # Add 10 future waypoints to the match list
    for i in range(80):
        idx = index + 10 + i * 5
        idx %= len(sim_map['x'])
        px.append(sim_map['x'][idx])
        py.append(sim_map['y'][idx])

    # Fit the spline
    sp = util.getSplineCoeffs(px,py)
    # print(px,py)
    # print(s,d,start_x,start_y, util.convertFromFrenet(s, d, map_s, sim_map['x'], sim_map['y']))
    
    pts = []
    fx = []
    fy = []

    s = np.arange(0, sp.s[-1], 2)
    count = 0
    for i in s:
        ix,iy = util.fitX(i,sp)
        fx.append(ix)
        fy.append(iy)
        if count > range(200):
            break
        count += 1
        
    # Add more pre-vehicle map waypoints for a better graph
    for i in range(400):
        idx = index - 5 - i * 5
        if idx < 0:
            idx += len(sim_map['x'])
        px.append(sim_map['x'][idx])
        py.append(sim_map['y'][idx])
        idx = index + 5 + i * 5
        idx %= len(sim_map['x'])
        px.append(sim_map['x'][idx])
        py.append(sim_map['y'][idx])

    fig = plt.figure()
    plt.scatter(px,py,c='blue')
    plt.scatter(fx,fy,c='red', marker='+')
    plt.scatter(start_x,start_y, linewidths=6, c='green', marker="+")
    plt.ylim([400, 3200])
    plt.xlim([100, 2500])
    #plt.legend(['map_waypoints','spline_path','car'])
    name = 'Images/Spline_' + str(index) + '.png'
    fig.savefig(name)
    print(name)

    
def testSciPySpline():
    map_s = util.generateMapS(sim_map['x'],sim_map['y'])
    index = 200
    start_x = sim_map['x'][index] - 3.24
    start_y = sim_map['y'][index] + 1.6

    # Convert the start position to frenet coordinates
    s,d = util.convertToFrenet(start_x,start_y,0,sim_map['x'],sim_map['y'])
    print'X,Y', start_x, start_y, ' >> S,D', s, d
    print'S,D', s, d, ' >> X,Y', util.convertFromFrenet(s, d, map_s, sim_map['x'], sim_map['y'])

    # Create and the array of points to fit the spline to
    px = [start_x-1]
    py = [start_y-1]
    px = [start_x]
    py = [start_y]

    # Add 10 future waypoints to the match list
    for i in range(10):
        idx = index + i * 10
        idx %= len(sim_map['x'])
        px.append(sim_map['x'][idx])
        py.append(sim_map['y'][idx])

    # Fit the spline
    sp = util.getSplineCoeffs(px,py)

    print(s,d,start_x,start_y, util.convertFromFrenet(s, d, map_s, sim_map['x'], sim_map['y']))
    pts = []
    fx = []
    fy = []

    for i in range(0, 50):
        ns = s + 1.25 * i
        nd = d
        x,y = util.convertFromFrenet(ns, nd, map_s, sim_map['x'], sim_map['y'])
        #print(ns,nd,x,y)
        y_ = util.fitX(x,tkc)
        fy.append(y_)
        fx.append(x)
        print(ns,nd,x,y,y_)

    # Add more pre-vehicle map waypoints for a better graph
    if True:
        for i in range(10):
            idx = index - i * 10
            px.append(sim_map['x'][idx])
            py.append(sim_map['y'][idx])

    plt.figure()
    plt.scatter(px,py,c='blue')
    plt.plot(fx,fy,c='red')
    plt.scatter(start_x,start_y, linewidths=6, c='green', marker="+")
    plt.legend(['map_waypoints','spline_path','car'])
    plt.show()

    print(s,d)
    
    
#testGitSpline(1500)
#testGitSpline(3000)
#testGitSpline(3740)
#testGitSpline(4640)
#testGitSpline(5140)
#testGitSpline(6240)
#testGitSpline(6350)
#testGitSpline(6530)
testGitSpline(6558,start_x=1254.016, start_y=2952.879, theta=3.00073934697)
testGitSpline(6592,start_x=1218.983, start_y=2958.804)
#testGitSpline(6677)
#testGitSpline(6730)
testGitSpline(10825)
#for i in range(9000,10901,10):
#    testGitSpline(i)
for i in range(0,100,5):
    idx = i + 10901
    idx %= 10901
    testGitSpline(idx)









