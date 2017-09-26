import pid
import lowpass
import rospy
import tf
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704



class Controller(object):
    def __init__(self, decel_limit, accel_limit, max_steer_angle,
          max_lat_accel, min_speed, wheel_base, steer_ratio, vehicle_mass, wheel_radius):
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.max_steer_angle = max_steer_angle
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius

        self.throttle_pid = pid.PID(kp = 0.6, ki = 0.004, kd = 0.2, mn=decel_limit, mx=accel_limit)
        self.throttle_filter = lowpass.LowPassFilter(tau = 0.0, ts = 1.0)

        self.steer_pid = pid.PID(kp = 0.5, ki = 0.05, kd = 0.3, mn=-max_steer_angle, mx=max_steer_angle)
        self.steer_filter = lowpass.LowPassFilter(tau = 0.0, ts = 1.0)

        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.clk = rospy.get_time()

    def control(self,
        target_linear_velocity,
        current_linear_velocity,
        target_angular_velocity,
        current_angular_velocity,
        steer_cte,
        dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        # Get sample time
        t = rospy.get_time()
        dt = t - self.clk
        self.clk = t

        if not dbw_enabled:
            self.throttle_pid.reset()
            self.throttle_filter.ready = False
            self.steer_pid.reset()
            self.steer_filter.ready = False
            rospy.loginfo("RESET!!!!")
            return 0.0, 0.0, 0.0

        # rospy.loginfo("DBW_ENABLED!!!!")

        velocity_cte = target_linear_velocity - current_linear_velocity
        # steer_cte = target_angular_velocity - current_angular_velocity

        # rospy.loginfo('ctrl: steer_cte = {}, dt = {}'.format(steer_cte, dt))
        # rospy.loginfo('ctrl: velocity_cte = {}, dt = {}'.format(velocity_cte, dt))

        # Throtle PID
        throttle = self.throttle_pid.step(velocity_cte, dt)
        # rospy.loginfo('ctrl: throttle = {}'.format(throttle))
        throttle = self.throttle_filter.filt(throttle)
        # rospy.loginfo('ctrl: throttle_filtered = {}'.format(throttle))


        # Steer PID
        steer = self.steer_pid.step(steer_cte, dt)
        # rospy.loginfo('ctrl: steer = {}'.format(steer))
        # steer = self.steer_filter.filt(steer)
        # rospy.loginfo('ctrl: steer_filtered = {}'.format(steer))

        # Yaw Controller
        steering = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)
        # rospy.loginfo('ctrl: steering = {}'.format(steering))
        # steering = self.steer_filter.filt(steering)

        steer = self.steer_filter.filt(steer + steering)

        # steer = 0.0

        if throttle >= 0:
            brake = 0.0
        else:
            # Calc brake torque as torque = Vmass * dec * wheel_radius
            # ref http://sciencing.com/calculate-brake-torque-6076252.html
            brake = self.vehicle_mass * self.wheel_radius * (-1.0 * throttle) #  * self.decel_limit
            throttle = 0.0

        # Stop still on red light :) - prevents slow movement near zero speed
        if target_linear_velocity < 0.1:
            throttle = 0.0
            brake = self.vehicle_mass * self.wheel_radius * (-1.0 * self.decel_limit)

        # Return throttle, brake, steer
        return throttle, brake, steer
