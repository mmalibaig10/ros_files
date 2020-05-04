#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import numpy as np
import math
import rosbag
import sys
from roslib.message import get_message_class
from math import sqrt
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2 as pc2
topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray)
bearing_std_dev = 0.001  #std uncertainity in laser scan(rad)
least_sq_angle_thresh = 0.0001 #threshold to stop iterating least squares
least_sq_angle_thresh = 0.0001
max_line_gap = 0.4
min_line_length = 0.5
min_line_points = 9
min_range = 0.4
min_split_dist = 0.05
outlier_dist = 0.05

range_std_dev = 0.02

x = 0
y = 0
k = 0
def callback_formulae(msg):


    ang_min = msg.angle_min #start angle of the scan
    ang_max = msg.angle_max   #end angle of the scan
    ang_incre = msg.angle_increment #angular distace between the measurements
    min_val = msg.range_min
    max_val = msg.range_max
    print("\n minimum range value is: {}".format(min_val))
    print("\n maximum range value is: {}".format(max_val))
    print("\n lenght of the ranges list is: {}".format(len(msg.ranges)))
    ranges = []
#####0degrees and 360degrees indicates forward ranges
####180degrees indicates backward ranges
####90degrees indicates left ranges
####270degrees indicates right rangles
    ranges = msg.ranges
    theta1 = math.cos(90)
    theta2 = math.sin(90)

    #x = ranges*theta1
    #y = ranges*theta2
    x = []
    y = []
    k = []
    for r in ranges:
        #x.append(r*math.cos(theta))
        x.append(r*theta1)
        #y.append(r*math.sin(theta))
        y.append(r*theta2)
        print("value of x", x)
        print("value of y", y)
        k.append(x+y) # Hough Transformation
        print("values of k", k)
        euclidean_distance(x, y, ranges)

        split(x, y, ranges)

def euclidean_distance(x, y, ranges):
        distance = 0.0
        for i in range(len(x)-1):
	       distance += (x[i] - y[i])**2
        return sqrt(distance)
        row0 = ranges[0]
        for row in ranges:
           distance = euclidean_distance(row0, row)
        print("distance", distance)
    # distance for each record in the dataset as a tuple,
    #sort the list of tuples by the distance (in descending order) and then retrieve the neighbors.

def get_neighbors(x, y, ranges, num_neighbors):
	    distances = list()
	    for y in x:
		   dist = euclidean_distance(y, x)
		   distances.append((x_row, dist))
	    distances.sort(key=lambda tup: tup[1])
	    neighbors = list()

	    for i in range(num_neighbors):
		   neighbors.append(distances[i][0])
	    return neighbors
        #neighbors = get_neighbors(ranges, ranges[0], 66.0)
        #for neighbor in neighbors:
	    print("neighbors", neighbors)
def split(x, y, ranges):
    #x = (end[0] - start[0]) / float(segments)
    #y = (end[1] - start[1]) / float(segments)
    points = []
    for i in range(1, ranges):
         points.append([start[0] + i * x, start[1] + i * y])
    return [start] + points + [end]
    print split([0, 20], [20, 75], 5)



def main():
    rospy.init_node('scan_values')
    rospy.Publisher ("/marker", pc2, split)
    #sub = rospy.Subscriber('/scan', LaserScan, callback_formulae)
    rospy.Subscriber('/scan', LaserScan, callback_formulae)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.logerror("Cannot run node!!")
