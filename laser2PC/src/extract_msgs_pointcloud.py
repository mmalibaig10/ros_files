#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
#from sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
import numpy as np
#mport pcl
import math
import time
import rosbag
import sys
from roslib.message import get_message_class
import pandas as pd


def callback(data):
    pc = ros_numpy.numpify(data)
    points = np.zeros((pc.shape[0],3))
    points[:,0] = pc ['X']
    points[:,1] = pc ['y']
    points[:,2] = pc ['z']
    p = pcl.PointCloud(np.array(points, dtype=np.float32))
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber("/laserPointCloud",PointCloud2, callback )
    rospy.spin()
