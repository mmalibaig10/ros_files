#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PoseStamped, PointStamped
import numpy as np
import math
import rosbag
import sys
from roslib.message import get_message_class
import pandas as pd
import time
import tf2tf2_ros
import tf2_geometry_msgs
import sys, select, terminos, tty
from math import cos, sin


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


rospy.init_node('scan_values')

sub = rospy.Subscriber('/scan', LaserScan, callback)
