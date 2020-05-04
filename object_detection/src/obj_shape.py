# import the necessary packages
import cv2
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import numpy as np
import math
import rosbag


	
	def detect(msg):
             c = []
             c = msg.ranges
             #while True:
              # _, c = cap.read()
               #hsv = cv2.cvtcolor(c, cv2.COLOR_BRG2HSV)
                
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)
                # if the shape is a triangle, it will have 3 vertices
		if len(approx) == 3:
			shape = "triangle"
		# if the shape has 4 vertices, it is either a square or
		# a rectangle
		elif len(approx) == 4:
			# compute the bounding box of the contour and use the
			# bounding box to compute the aspect ratio
			(x, y, w, h) = cv2.boundingRect(approx)
			ar = w / float(h)
			# a square will have an aspect ratio that is approximately
			# equal to one, otherwise, the shape is a rectangle
			shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
		# if the shape is a pentagon, it will have 5 vertices
		elif len(approx) == 5:
			shape = "pentagon"
		# otherwise, we assume the shape is a circle
		else:
			shape = "circle"
		# return the name of the shape
		return shape
                sift(c)
       def sift(c):
               
               # select your source sift, surf or orb.cd 
               sift = cv2.xfeatures2d.SIFT_create()
               #surf = cv2.xfeatures2d.SURF_create()
               #orb = cv2.ORB_create(nfeatures=2800)

               keypoints_sift, descriptors = sift.detectAndCompute(c, None)
               #keypoints_surf, descriptors = surf.detectAndCompute(c, None)
               #keypoints_orb, descriptors = orb.detectAndCompute(c, None)
               c = cv2.draqkeypoints(imshow, keypoints, None)

       def ():
		rospy.init_node('scan_values')
                rospy.Publisher ("/marker", pc2,queue_size=1)
                rospy.publisher ("7markerArray" pc2,)
                rospy.Subscriber('/scan', LaserScan, detect)
                rospy.spin()
