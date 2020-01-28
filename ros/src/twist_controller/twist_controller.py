
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.kp = 0.001
        self.kd = 0.100
        self.ki = 0.000
        self.pid_thr = PID(self.kp, self.ki, self.kd, 1.0, 0.0)
        self.dt = rospy.get_time()

    def control(self, prop_lin_vel, prop_ang_vel, curr_lin_vel, dbw_enabled, pose_z):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        # Get dt
        sample_time = self.dt - rospy.get_time()
        dt = rospy.get_time()

        # Get linear error
        rospy.logwarn("prop_lin_vel = %s, curr_lin_vel = %s, prop_ang_vel = %s, pose_z = %s\n", prop_lin_vel, curr_lin_vel, prop_ang_vel, pose_z)
        lin_err = curr_lin_vel - prop_lin_vel
        
        # Must obtain proposed angle from proposed angular velocity
        # angular velocity omega = (th1 - th0) / (t1-t0) = dtheta / dt # grc.nasa.gov/www/k-12/airplane/angdva.html
        # Therefore angular proposed position th1 = omega*dt + th0 --> prop_ang = prop_ang_vel * dt + pose_z
        prop_ang = prop_ang_vel * (dt / 10000000000.0) + pose_z
        rospy.logwarn("prop_ang = %s, pose_z = %s\n", prop_ang, pose_z)
        ang_err = pose_z - prop_ang
        
        # Reset PID if needed
        if not dbw_enabled:
           self.pid_thr.reset()
           return 0.0, 0.0, 0.0

        # Advance PID
        thr = self.pid_thr.step(lin_err, dt)
        
        # Low pass should be here somewhere or before pid control
        
        # Break - hipass
        brk = 0
        
        # Steering, P controller for the time being - should be replaced with YawController
        ster = -ang_err * 20
        
        return thr, brk, ster
