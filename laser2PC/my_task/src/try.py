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
import pandas as pd


def callback(msg):
    ang_min = msg.angle_min #start angle of the scan
    ang_max = msg.angle_max   #end angle of the scan
    ang_incre = msg.angle_increment #angular distace between the measurements
    min_val = msg.range_min
    max_val = msg.range_max
    print("\n minimum range value is: {}".format(min_val))
    print("\n maximum range value is: {}".format(max_val))
    print("\n lenght of the ranges list is: {}".format(len(msg.ranges)))
    values = []
    values = msg.ranges
    print('s1 [0]')     #value front-direction laser beam
    print values[0]  #print the distance to an abstacle infront of the robot.the sensor returns a vector
                       #946 values, being the inital value the corresponding to the front of robot

    print('s2 [90]')
    print values[90]

    print ('s3 [180]')
    print values[180]

    print ('s4 [270]')
    print values[270]
    # for value in values:
    #     if not value < min_val and value > max_val:
    #         print(value)
        #print(values)
    print(np.array(values).shape)
    a = np.array(values)
        # Following line splits the array into subgroups on the basis of equal x-axis elements

    a = np.split(a, np.unique(a[:, 0], return_index=True)[1][1:], axis=0)
    i = 0
    # filteredList will initially contain the first element of the array's first sub group

    filteredList = np.reshape(np.asarray(a[0][0]), (-1, 2)) # filteredList = [[581 925]]

    while not i == len(a) - 1:
        if len(a[i + 1]) > 1:
            # Following line calculates the euclidean distance between current point and the points in the next group
            min_dist_point_addr = np.argmin(np.linalg.norm(filteredList[i] - a[i + 1], axis=1))

            # Next group is reassigned with the element to whom the distance is the least
            a[i + 1] = a[i + 1][min_dist_point_addr]
        # The element is concatenated to filteredList
        filteredList = np.concatenate((filteredList, np.reshape((a[i+1]), (1, 2))), axis=0)
        i += 1
        print (filteredList)

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
