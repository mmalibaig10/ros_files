import cv2
import numpy as np
#convert bag file to cv2.IMREAD_GRAYSCALE
img = (filename(img),cv2.IMREAD_GRAYSCALE)

#The three features are almost same, implement only one.
sift = cv2.xfeatures2d.SIFT_create()

surf = cv2.xfeatures2d.SURF_create()

#for orb we have to declare number of points in it.we can increase or decrease 
orb = cv2.ORB_create(nfeatures=2800)


keypoints_sift, descriptors = sift.detectAndCompute(scan msg, None)
#keypoints_surf, descriptors = surf.detectAndCompute(scan msg, None)
#keypoints_orb, descriptors = orb.detectAndCompute(scan msg, None)

# Now that image with above keypoints

img = cv2.draqkeypoints(img, keypoints, None)

cv2.imshow("image", img)
cv2.waitkey(0)
#cv2.destroyALLWindows() 
