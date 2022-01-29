#!/usr/bin/env python
"""
This module implements the DBWNode.
"""


# STL
import math

# ROS imports
import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped

# Local imports
from twist_controller import TwistController


class DBWNode:
    """This class implements the drive-by-wire node."""

    _frequency = None
    _enabled = None

    _curr_velocity = None
    _curr_linear_velocity = None
    _curr_angular_velocity = None

    _twist_controller = None

    def __init__(self, frequency=50, enabled=False):
        if frequency is not None:
            self._frequency = frequency
        if enabled is not None:
            self._enabled = enabled
        self._initialize_node()

    def _initialize_node(self):
        rospy.init_node("dbw_node")

        vehicle_mass = rospy.get_param("~vehicle_mass", 1736.35)
        fuel_capacity = rospy.get_param("~fuel_capacity", 13.5)
        brake_deadband = rospy.get_param("~brake_deadband", 0.1)
        decel_limit = rospy.get_param("~decel_limit", -5)
        accel_limit = rospy.get_param("~accel_limit", 1.0)
        wheel_radius = rospy.get_param("~wheel_radius", 0.2413)
        wheel_base = rospy.get_param("~wheel_base", 2.8498)
        steer_ratio = rospy.get_param("~steer_ratio", 14.8)
        max_lat_accel = rospy.get_param("~max_lat_accel", 3.0)
        max_steer_angle = rospy.get_param("~max_steer_angle", 8.0)

        self._steer_pub = rospy.Publisher("/vehicle/steering_cmd", SteeringCmd, queue_size=1)
        self._throttle_pub = rospy.Publisher("/vehicle/throttle_cmd", ThrottleCmd, queue_size=1)
        self._brake_pub = rospy.Publisher("/vehicle/brake_cmd", BrakeCmd, queue_size=1)

        rospy.Subscriber("/current_velocity", TwistStamped, self._velocity_cb)
        rospy.Subscriber("/twist_cmd", TwistStamped, self._twist_cmd_cb)

        # This value is bound to the manual mode in simulator
        rospy.Subscriber("/vehicle/dbw_enabled", Bool, self._dbw_enabled_cb)

        self._twist_controller = TwistController(
            vehicle_mass=vehicle_mass,
            fuel_capacity=fuel_capacity,
            brake_deadband=brake_deadband,
            decel_limit=decel_limit,
            accel_limit=accel_limit,
            wheel_radius=wheel_radius,
            wheel_base=wheel_base,
            steer_ratio=steer_ratio,
            max_lat_accel=max_lat_accel,
            max_steer_angle=max_steer_angle,
        )

        self._spin()

    def _velocity_cb(self, msg):
        self._curr_velocity = msg.twist.linear.x

    def _twist_cmd_cb(self, msg):
        self._curr_linear_velocity = msg.twist.linear.x
        self._curr_angular_velocity = msg.twist.angular.z

    def _dbw_enabled_cb(self, enabled):
        """Cache `dbw_enabled`."""
        self._enabled = enabled

    def _spin(self):
        """Start the node."""
        rate = rospy.Rate(self._frequency)
        while not rospy.is_shutdown():
            if not None in (
                self._curr_velocity,
                self._curr_linear_velocity,
                self._curr_angular_velocity,
            ):
                if self._enabled:
                    throttle, brake, steering = self._twist_controller.control(
                        self._curr_velocity,
                        self._curr_linear_velocity,
                        self._curr_angular_velocity,
                        self._enabled,
                    )
                    self._publish(throttle, brake, steering)

            rate.sleep()

    def _publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self._throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self._steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self._brake_pub.publish(bcmd)


if __name__ == "__main__":
    try:
        DBWNode()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start waypoint updater node.")
