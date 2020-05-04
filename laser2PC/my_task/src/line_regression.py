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

    global x,y
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
    theta = theta1+theta2
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
        cov_mat = np.cov((k, theta))
