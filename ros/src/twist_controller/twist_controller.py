from pid import PID
import rospy
from yaw_control import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704
VERBOSE = false


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        #, , , , , , , , , 
        self.vehicle_mass = args[0]
        self.fuel_capacity = args[1]
        self.brake_deadband = args[2]
        self.decel_limit = args[3]
        self.accel_limit = args[4]
        self.wheel_radius = args[5]
        self.wheel_base = args[6]
        self.steer_ratio = args[7]
        self.max_lat_accel = args[8]
        self.max_steer_angle = args[9]

        # todo these need to be updated
        self.pid_vel = PID(0.1, 0.01, 0.1)
        self.yaw_control = YawController(self.wheel_base, self.steer_ratio, 0.0, self.max_lat_accel, self.max_steer_angle )

        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        linear_velocity = args[0]
        angular_velocity = args[1]
        current_velocity = args[2]
        is_dbw_enabled = args[3]
        time_elapsed = args[4]

        # the velocity difference is needed for the pid for velocity.
        velocity_difference = linear_velocity - current_velocity
        velocity = self.pid_vel.step(velocity_difference, elapsed_time)

        #todo, may need to double check this... the goal is to set the brake to the inverse of the throttle
        brake = math.fabs(min(0.0, velocity))

        # the steering difference is needed for the pid for steering angle.
        steering = self.yaw_control.get_steering(linear_velocity, angular_velocity, current_velocity)

        # because we need to troubleshoot.
        if (VERBOSE):
        	rospy.logerr("velocity = {}".format(velocity))
        	rospy.logerr("steering = {}".format(steering))
            rospy.logerr("brake = {}".format(brake))       



        return velocity, brake, steering
