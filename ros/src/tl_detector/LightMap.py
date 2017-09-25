from geometry_msgs.msg import Pose
import rospy
import util

LIGHT_SHIFT_UNKNOWN = 0
LIGHT_GETTING_CLOSER = 1
LIGHT_GETTING_FARTHER = 2

FLOAT_THRESHOLD = 0.0001

def floats_equal(a, b):
    return abs(a - b) < FLOAT_THRESHOLD

def friendly_name(shift):
    shift_dict = {LIGHT_SHIFT_UNKNOWN: 'LIGHT_SHIFT_UNKNOWN',
                  LIGHT_GETTING_CLOSER: 'LIGHT_GETTING_CLOSER',
                  LIGHT_GETTING_FARTHER: 'LIGHT_GETTING_FARTHER'}
    return shift_dict[shift]

class Light:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.distance_from_car = None
        self.shift_relative_to_car = LIGHT_SHIFT_UNKNOWN

    def get_as_pose(self):
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        return pose

    def update_distance_from_car(self, car_pose):
        new_distance = util.euclidean_distance(self.x,
                                               self.y,
                                               car_pose.position.x,
                                               car_pose.position.y)

        if self.distance_from_car:
            if floats_equal(new_distance, self.distance_from_car):
                # No change in shift_relative_to_car
                pass
            elif new_distance < self.distance_from_car:
                self.shift_relative_to_car = LIGHT_GETTING_CLOSER
            elif new_distance > self.distance_from_car:
                self.shift_relative_to_car = LIGHT_GETTING_FARTHER

        self.distance_from_car = new_distance

    def is_closer(self, other):
        if ((other.shift_relative_to_car == LIGHT_GETTING_FARTHER or
             other.shift_relative_to_car == LIGHT_SHIFT_UNKNOWN)
            and self.shift_relative_to_car == LIGHT_GETTING_CLOSER):
            return True
        else:
            return (self.shift_relative_to_car == LIGHT_GETTING_CLOSER
                    and not floats_equal(self.distance_from_car, other.distance_from_car)
                    and self.distance_from_car < other.distance_from_car)

    def __repr__(self):
        return '(x,y): ({},{}), dist: {}, shift: {}'.format(self.x, self.y, self.distance_from_car, friendly_name(self.shift_relative_to_car))

def get_light_pose(light_position):
    light_pose = Pose()
    light_pose.position.x = light_position[0]
    light_pose.position.y = light_position[1]
    return light_pose

class LightMap:
    def __init__(self, waypoints, light_positions):
        self.waypoints = waypoints
        self.lights = []
        for light_position in light_positions:
            light_pose = get_light_pose(light_position)
            self.lights.append(Light(light_position[0],
                                     light_position[1]))
        self.first_update = True

    def get_closest_waypoint_to_upcoming_light(self, car_pose):
        if self.first_update:
            self.first_update = False
            return -1

        if not car_pose:
            return -1

        closest_waypoint_to_car = self._get_closest_waypoint_to_car(car_pose)

        # find the closest visible traffic light (if one exists)
        closest_light = self.lights[0]
        for light in self.lights:
            light.update_distance_from_car(closest_waypoint_to_car.pose)
            if light.is_closer(closest_light):
                closest_light = light

        # rospy.loginfo('Closest light: {}'.format(closest_light))

        light_pose = closest_light.get_as_pose()

        closest_waypoint_to_light = util.get_closest_waypoint(light_pose, self.waypoints)

        return closest_waypoint_to_light

    def _get_closest_waypoint_to_car(self, car_pose):
        closest_waypoint_index = util.get_closest_waypoint(car_pose.pose, self.waypoints)
        return self.waypoints.waypoints[closest_waypoint_index].pose
