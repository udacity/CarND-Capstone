import matplotlib.pyplot as plt
import numpy as np
import rospkg
import os.path

Plot_Lateral_Control = False
Plot_Longitudinal_Control = True

#data from waypoint updater node
log_dir = rospkg.get_log_dir() + "/latest"
log_dir = os.path.realpath(log_dir)
log_file = log_dir + "/" + "waypoint_updater.csv"
wu_time, x, y, wp, cx, cy = np.loadtxt(log_file,  delimiter=',', skiprows=1, unpack = True)
#data from dbw node
log_file = log_dir + "/" + "dbw_node.csv"
throttle_p_effort, throttle_i_effort, throttle_d_effort, brake_p_effort, brake_i_effort, brake_d_effort, velocity_error, DT, latchBrake, dbw_time, target_linear_velocity, target_angular_velocity,current_linear_velocity, current_angular_velocity, dbw_status, throttle, brake, steering,  = np.loadtxt(log_file,  delimiter=',', skiprows=1, unpack = True)

#time align data from different ros nodes
start_time = wu_time[0]
dbw_time = dbw_time - start_time
wu_time = wu_time - start_time
dbw_time = dbw_time*1E-9
wu_time = wu_time*1E-9

################ lateral controller subplot ##################
if Plot_Lateral_Control:
    f, axx = plt.subplots(2,3)
    f.suptitle('Lateral Control')
    axx[0, 0].plot(dbw_time, target_angular_velocity, label="setpoint")
    axx[0, 0].plot(dbw_time, current_angular_velocity, label="actual")
    axx[0, 0].set_title('anguar velocity')
    axx[0, 0].legend()

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

    axx[1, 0].plot(dbw_time, steering)
    axx[1, 0].set_title('steer_command')

    axx[1, 1].plot(dbw_time, dbw_status, label="dbw")
    axx[1, 1].set_title('dbw enabled')
    axx[1, 1].grid()

    axx[1, 2].set_title('waypoint indices')
    axx[1, 2].plot(wu_time, wp, label="waypoint")

##########  longitudinal control subplot ##################
if Plot_Longitudinal_Control:
    f, ax = plt.subplots(2,3)
    f.suptitle('Longitudinal Control')
    ax[0, 0].plot(dbw_time, target_linear_velocity, label="setpoint")
    ax[0, 0].plot(dbw_time, current_linear_velocity, label="actual")
    ax[0, 0].legend(loc=4)
    ax[0, 0].grid()
    ax[0, 0].set_title('speed')

    ax[0, 1].plot(dbw_time, brake, label="brake_cmd")
    ax[0, 1].set_title('brake cmd')
    ax[0, 1].grid()

    ax[0, 2].plot(dbw_time, brake_p_effort, label="p")
    ax[0, 2].plot(dbw_time, brake_i_effort, label="i")
    ax[0, 2].plot(dbw_time, brake_d_effort, label="d")
    ax[0, 2].grid()
    ax[0, 2].set_title('Brake PID')
    ax[0, 2].legend(loc=1)

    #Calculate Linear, longitudinal acceleration
    span = 50
    current_linear_accel = np.zeros(len(dbw_time)-span)
    for i in range (len(current_linear_velocity)-span):
        current_linear_accel[i]=((current_linear_velocity[i+span]-current_linear_velocity[i])/(dbw_time[i+span]-dbw_time[i]))
    ax[1, 0].plot(dbw_time[span:], current_linear_accel, label="actual")
    ax[1, 0].grid()
    ax[1, 0].set_title('acceleration')

    ax[1, 1].plot(dbw_time, throttle, label="throttle cmd")
    ax[1, 1].set_title('throttle cmd')
    ax[1, 1].grid()

    ax[1, 2].plot(dbw_time, throttle_p_effort, label="p")
    ax[1, 2].plot(dbw_time, throttle_i_effort, label="i")
    ax[1, 2].plot(dbw_time, throttle_d_effort, label="d")
    ax[1, 2].grid()
    ax[1, 2].set_title('Throttle PID')
    ax[1, 2].legend(loc=1)

plt.show()