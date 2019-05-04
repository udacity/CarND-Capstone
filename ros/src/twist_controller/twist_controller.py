

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass,fuel_capacity,brake_deadband,decel_limit,accel_limit,wheel_radius,wheel_base,steer_ratio,max_lat_accel,max_steer_angle):
        self.yaw_controller = YawController(wheel_base,steer_ratio,0.1,max_lat_accel,max_steer_angle)
        kp=0.3
        kd=0.
        ki=0.1
        mn=0. # min throttle value
        mx=0.2 # max throttle value
        self.throttle_pid = PID(kp, ki, kd, mn, mx)

        self.throttle_controller=PID(kp,ki,kd,mn,mx)
        tau=0.5 # cut off frequency
        ts=.02 # sample time
        self.vel_lpf=LowPassFilter(tau,ts)
        self.vehicle_mass=vehicle_mass
        self.fuel_cap=fuel_capacity
        self.brake_dea=brake_deadband
        self.decel_limit=decel_limit
        self.accel_limit=accel_limit
        self.wheel_radius=wheel_radius
        self.last_time=rospy.get_time()

    def control(self, current_vel, dbw_enabled, target_linear_vel, target_angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        if not dbw_enabled:
            self.throttle_pid.reset()
            return 0., 0., 0.

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time


        current_vel=self.vel_lpf.filt(current_vel)

        steer = self.yaw_controller.get_steering(target_linear_vel, target_angular_vel, current_vel)
        vel_error = target_linear_vel - current_vel
        throttle = self.throttle_pid.step(vel_error, sample_time)

        brake = 0

        if linear_vel==0. and current_vel<0.2:
            throttle=0
            brake=400
        elif throttle <.1 and velocity_error<0:
            throttle=0
            decel=max(velocity_error,self.decl_limit)
            brake=abs(decel)*self.vehicle_mass*self.wheel_radius

        # return 1., 0., 0.
        return throttle, brake, steer
