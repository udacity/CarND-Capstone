import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
STOP_VELOCITY = 0.277778 # 1 km/h

class TwistController(object):
    def __init__(self, vehicle_mass, fuel_capacity,
                 brake_deadband, decel_limit, accel_limit,
                 wheel_radius, wheel_base,
                 steer_ratio, max_lat_accel,
                 max_steer_angle):
        # init members
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        # init controllers
        self.velocity_pid = PID(0.65, 0.0, 0.0,
                                mn=decel_limit, mx=accel_limit)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 1,
                                            max_lat_accel, max_steer_angle)

    def control(self, twist, current_velocity, dt):
        """
        twist is for the desired velocity
        """
        # Return throttle, brake, steer
        velocity_cte = abs(twist.twist.linear.x) - current_velocity.twist.linear.x
        # much discussion that there might be a bug in pure_pursuit (waypoint_follower)
        # that the twist.twist.linear might be negative:
        # https://carnd.slack.com/archives/C6NVDVAQ3/p1512659466000795
        # take abs to avoid the problem, assuming the magnitude is more reliable
        # Verified with casual running the simulator, it showed no negative impact.

        acceleration = self.velocity_pid.step(velocity_cte, dt)
        steer = self.yaw_controller.get_steering(twist.twist.linear.x,
                                                 twist.twist.angular.z,
                                                 current_velocity.twist.linear.x)

        throttle = 0.0
        brake = 0.0

        # Note that throttle values passed to publish should be in the range 0 to 1.
        if twist.twist.linear.x < STOP_VELOCITY:
            brake = self.calc_torque(abs(self.decel_limit))
        else:
            if acceleration < 0.0:
                deceleration = -acceleration
                if deceleration < self.brake_deadband:
                    brake = 0.0
                else:
                    brake = self.calc_torque(deceleration)
            else:
                throttle = acceleration

        return throttle, brake, steer

    def calc_torque(self, acceleration):

        return acceleration * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius
