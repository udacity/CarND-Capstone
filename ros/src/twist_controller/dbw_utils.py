"""
Utils class for dbw_node 
"""

import rospy
import math
import numpy as np

def get_steerig_cte(twist_waypoints, current_pose): 
	"""
	Using twist_waypoints, compute the expected (x,y) position of the car and compare it to the actual position. 
	:return - Cross Track Error, deviation from expected trajectory
	"""

	# Get 20 waypoints coordinates to fit polynomial.
	#points_x = [i.pose.pose.position.x for i in twist_waypoints]#[0:20]
	#points_y = [i.pose.pose.position.y for i in twist_waypoints]#[0:20]
	points_x = [i.linear.x for i in twist_waypoints][0:20]
	points_y = [i.linear.y for i in twist_waypoints][0:20]
	
	# Transformation into the vehicle system:
	points_x_car = []
	points_y_car = []
	
	# From quaternion to Euler angles:
	x = current_pose.orientation.x
	y = current_pose.orientation.y
	z = current_pose.orientation.z
	w = current_pose.orientation.w
	
	# Determine car heading:
	t3 = +2.0 * (w * z + x*y)
	t4 = +1.0 - 2.0 * (y*y + z*z)
	theta = math.degrees(math.atan2(t3, t4))
	
	# Coordinate transformation, from Udacity given equation. 
	for i in range(len(points_x)):
		car_x = (points_y[i]-current_pose.position.y)*math.sin(math.radians(theta))-(current_pose.position.x-points_x[i])*math.cos(math.radians(theta))
		car_y = (points_y[i]-current_pose.position.y)*math.cos(math.radians(theta))-(points_x[i]-current_pose.position.x)*math.sin(math.radians(theta))
		points_x_car.append(car_x)
		points_y_car.append(car_y)
	
	# Interpolate points in the vehicle coordinate system with a 3rd degree polynomial
	coeff_xy = list(reversed(np.polyfit(points_x_car, points_y_car, 3)))
	cte = 0
	for p, coeff in enumerate(coeff_xy):
		cte += coeff * (2.0 ** p)

	return cte


def get_throttle_cte(twist_waypoints, current_pose): 
	return 2