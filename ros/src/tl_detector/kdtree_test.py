#!/usr/bin/env python

import fileinput
import random
from kdtree import KDTree

# read in the points from a file specified on the command line. E.g.:
# $ ./kdtree_test.py ../../../data/sim_waypoints.csv
points = []
i=0
for line in fileinput.input():
    line_parts = line.split(',')
    points.append((float(line_parts[0]), float(line_parts[1]), int(i)))
    i += 1

# generate the K-D Tree from the points
kdtree = KDTree(points)

# pick a random point from the points
rand_index = random.randint(0, len(points))

# find the closest point
point = points[rand_index]
print ("randomly chose point {} at index {}".format(point, rand_index))
new_point = (point[0] + 2.0, point[1] + 7.0)
print ("tweaked x,y to be {}".format(new_point, rand_index))

closest = kdtree.closest_point(new_point)

print ("closest point to {} is {}".format(new_point, closest))
