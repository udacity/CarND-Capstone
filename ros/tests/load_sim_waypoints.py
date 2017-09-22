import csv
from styx_msgs.msg import Waypoint

CSV_HEADER = ['x', 'y', 'z', 'yaw']


def load_sim_waypoints():
    file_name = '../../data/sim_waypoints.csv'
    waypoints = []
    with open(file_name) as csv_file:
        reader = csv.DictReader(csv_file, CSV_HEADER)

        for wp in reader:
            p = Waypoint()
            p.pose.pose.position.x = float(wp['x'])
            p.pose.pose.position.y = float(wp['y'])
            p.pose.pose.position.z = float(wp['z'])
            waypoints.append(p)

    return waypoints
