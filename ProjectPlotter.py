import matplotlib.pyplot as plt
import numpy as np


#ff, proportional, integral, dbw_time, speed_setpoint, current_speed, error, throttle_command, steer_command, yawrate_setpoint, yawrate, dbw_bool = np.loadtxt('dbw.txt', unpack = True)
#wu_time, wp, cx, cy, x, y, count = np.loadtxt('waypointUpdater.txt', unpack = True)

#rospy.get_time(), car_x, car_y, closest_idx_waypoint, self.wps.waypoints[closest_idx_waypoint].pose.pose.position.x, self.wps.waypoints[closest_idx_waypoint].pose.pose.position.y

wu_time, x, y, wp, cx, cy = np.loadtxt('/home/student/.ros/waypoint_updater.csv',  delimiter=',', unpack = True)
velocity_error, DT, dbw_time, target_linear_velocity, target_angular_velocity,current_linear_velocity, current_angular_velocity, dbw_status, throttle, brake, steering,  = np.loadtxt('/home/student/.ros/dbw_node.csv',  delimiter=',', unpack = True) 



start_time = wu_time[0]
dbw_time = dbw_time - start_time
wu_time = wu_time - start_time



#lateral control
f, axx = plt.subplots(2,3)
f.suptitle('Lateral Control')
axx[0, 0].plot(dbw_time, target_angular_velocity, label="setpoint")
axx[0, 0].plot(dbw_time, current_angular_velocity, label="actual")
axx[0, 0].set_title('anguar velocity')
axx[0, 0].legend()
axx[1, 0].plot(dbw_time, steering)
axx[1, 0].set_title('steer_command')
axx[1, 1].plot(dbw_time, dbw_status, label="dbw")
axx[1, 1].set_title('dbw enabled')
axx[1, 1].grid()

axx[1, 2].set_title('waypoint indices')
axx[1, 2].plot(wu_time, wp, label="waypoint")
axx[0, 2].set_title('position')
axx[0, 2].plot(x,y, label="actual")
axx[0, 2].plot(cx,cy, label="setpoint")
axx[0, 2].legend()
i=0
for xy in zip(x, y):
    if i % 50 == 0:
        axx[0, 2].annotate( '%.2f' % wu_time[i], xy=xy, textcoords='data')
    i += 1
i=0
for xy in zip(cx, cy):
    if i % 50 == 0:
        axx[0, 2].annotate( '%.2f' % wu_time[i], xy=xy, textcoords='data')
    i += 1

#longitudinal control
f, ax = plt.subplots(2,2)
ax[0, 0].plot(dbw_time, target_linear_velocity, label="setpoint")
ax[0, 0].plot(dbw_time, current_linear_velocity, label="actual")
ax[0, 0].legend()
ax[0, 0].grid()
ax[0, 0].set_title('speed')
ax[1, 0].plot(dbw_time, throttle, label="throttle cmd")
ax[1, 0].set_title('throttle cmd')
ax[1, 0].set_ylim(0, 2)
ax[1, 0].grid()
ax[0, 1].plot(dbw_time, brake, label="brale_cmd")
ax[0, 1].set_title('brale cmd')
ax[0, 1].grid()
ax[1, 1].plot(dbw_time, dbw_status, label="dbw")
ax[1, 1].set_title('dbw enabled')
ax[1, 1].grid()



#f, ax2= plt.subplots(2, 1)
#ax2[0].plot(dbw_time,proportional, label="proportional")
#ax[0].plot(dbw_time, error, label="error")
#ax2[0].plot(dbw_time,integral, label="integral")
#ax2[0].plot(dbw_time, throttle_command, label="throttle")
#ax2[0].plot(dbw_time, ff, label="ff")
#ax2[0].plot(dbw_time, dbw_bool, label="dbw")
#ax2[0].legend()
#ax2[1].plot(dbw_time,error, label="error")
#ax2[1].legend()
#ax2[1].grid()

plt.show()
