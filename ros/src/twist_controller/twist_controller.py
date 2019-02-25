import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, car_prm):
        # TODO: Implement
        self.yaw_controller = YawController(car_prm.wheel_base, car_prm.steer_ratio, 0.1, car_prm.max_lat_accel, car_prm.max_steer_angle)

        self.kp = 0.85
        self.ki = 0.
        self.kd = 0.08
        self.decel_limit = car_prm.decel_limit
        self.accel_limit = car_prm.accel_limit
        self.throttle_controller = PID(self.kp, self.ki, self.kd, self.decel_limit, self.accel_limit)

        #tau = .3
        #ts = 0.02
        self.vel_lpf = LowPassFilter(0.3, 0.02)
        self.st_lpf = LowPassFilter(0.02, 0.20)
        self.th_lpf = LowPassFilter(0.3, 0.02)

        self.vehicle_mass = car_prm.vehicle_mass
        self.wheel_radius = car_prm.wheel_radius

        self.last_vel = 0
        self.last_time = rospy.get_time()

    def control(self, curr_vel, twist_cmd_new, dbw_enbl):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enbl:
            self.throttle_controller.reset()
            return 0., 0., 0.

        #Filter the current Velocity
        curr_vel_flt = self.vel_lpf.filt(curr_vel.twist.linear.x)
        
        #Fetch Linear and Angular componenets from twist_cmd
        lin_vel = abs(twist_cmd_new.twist.linear.x)
        ang_vel = twist_cmd_new.twist.angular.z

        #estimate the velocity error
        vel_err = lin_vel - curr_vel_flt
        self.last_vel = curr_vel_flt
        
        #call yaw controller for steering estimation
        steering = self.yaw_controller.get_steering(lin_vel, ang_vel, curr_vel_flt)
        
        #steer_cte = self.st_controller.step()
        steering = self.st_lpf.filt(steering)
        
        #determine the sample time
        curr_time = rospy.get_time()
        sample_time = curr_time - self.last_time
        self.last_time = curr_time
        
        #call throttle controller for throttle estimation
        throttle = self.throttle_controller.step(vel_err, sample_time)
        throttle = self.th_lpf.filt(throttle)
        brake = 0

        if lin_vel == 0 and curr_vel_flt < 0.1:
            throttle = 0
            brake = 400
        elif throttle < 0.1 and vel_err < 0:
            throttle = 0
            decel = max(vel_err, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius


        return throttle, brake, steering
