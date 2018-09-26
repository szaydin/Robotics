#!/usr/bin/env python

# Based on:
#	https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_feature_homography/py_feature_homography.html#py-feature-homography
#	https://docs.opencv.org/3.1.0/d7/d53/tutorial_py_pose.html
#	https://docs.opencv.org/3.1.0/d1/d89/tutorial_py_orb.html
#	https://github.com/n17r4m/kobuki/blob/master/demo6/src/part3.py

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import math
import time
from matplotlib import pyplot as plt
import os

from numpy import cross, eye, dot
from scipy.linalg import expm3, norm
# For emblem
MIN_MATCH_COUNT = 25

# For AR
# MIN_MATCH_COUNT = 10

class Homography:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.cam_info_sub = rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self.info_cb)
		self.img_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.img_cb)

		# Load the target image
		self.directory_path = os.path.dirname(os.path.realpath(__file__))

		self.target_image = cv2.imread(self.directory_path + "/emblem.png", 0)

		self.imgpts = np.zeros((3, 1, 2), dtype=np.int)
		self.imgpts2 = np.zeros((3, 1, 2), dtype=np.int)
		# Initiate STAR detector
		self.orb = cv2.ORB()
		# find the keypoints with ORB
		self.kp = self.orb.detect(self.target_image,None)
		# compute the descriptors with ORB
		self.kp, self.des = self.orb.compute(self.target_image, self.kp)
		self.des = np.float32(self.des)
		self.eye = np.identity(3)
		self.axis = np.float32([[30,0,0], [0,30,0], [0,0,-30],[0,0,0]]).reshape(-1,3)
		self.axis2 = np.float32([[-30,0,0], [0,-30,0], [0,0,30]]).reshape(-1,3)
		self.center = None

		self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
		self.K = None
		self.D = None

		self.prev_imgpts = None
		self.prev_rect = None

	def info_cb(self, msg):
		self.K = np.array(msg.K).reshape(3,3)
		self.D = np.array(msg.D)

	def img_cb(self, msg):
		img  = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		img  = img[50:500]
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


		orb = cv2.ORB()
		kp = orb.detect(gray,None)
		kp, des = self.orb.compute(gray, kp)
		des = np.float32(des)

		FLANN_INDEX_KDTREE = 0
		index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
		search_params = dict(checks = 50)

		flann = cv2.FlannBasedMatcher(index_params, search_params)
		matches = flann.knnMatch(self.des, des, k=2)


		# store all the good matches as per Lowe's ratio test.
		good = []
		for m,n in matches:
			if m.distance < 0.75*n.distance:
				good.append(m)

		if len(good)>MIN_MATCH_COUNT:
			src_pts = np.float32([ self.kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
			dst_pts = np.float32([ kp[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

			M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
			matchesMask = mask.ravel().tolist()

			h,w = self.target_image.shape

			rect = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
			rect3d = np.float32([ [0,0,0],[0,h-1,0],[w-1,h-1,0],[w-1,0,0] ]).reshape(-1,1,3)
			rect = cv2.perspectiveTransform(rect,M)

			# Center of rectangle = ( (x1 + x2) / 2 , (y1 + y2) / 2)
			# where the two points are diagonals of each other
			center_x = int((rect[0][0][0] + rect[2][0][0]) / 2)
			center_y = int((rect[0][0][1] + rect[2][0][1]) / 2)

			x_offset = 9
			y_offset = 1

			self.center = (center_x, center_y)
			print "center:", self.center[0] + x_offset

			img2 = cv2.polylines(gray,[np.int32(rect)],True,255,3, cv2.CV_AA)

			dst2 = dst_pts[matchesMask].reshape(dst_pts.shape[0], 2)
			src2 = src_pts[matchesMask].reshape(dst_pts.shape[0], 2)

			rvecs, tvecs, inliers = cv2.solvePnPRansac(rect3d, rect, self.K, self.D)

			imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.K, self.D)
			# imgpts2 = cv2.projectPoints(self.axis2, rvecs, tvecs, self.K, self.D)

			# To store imgpt[i] for when we lose track of object
			self.prev_imgpts = imgpts
			self.prev_rect = rect

			# Note: imgpts[3] contains the x and y coordinates of the top-left corner of the rectangle
			#		which surrounds the found object.
			# image = cv2.line(img, tuple(imgpts[3].ravel()), tuple(imgpts[0].ravel()), (255,0,0), 5)
			# image = cv2.line(img, tuple(imgpts[3].ravel()), tuple(imgpts[1].ravel()), (0,255,0), 5)
			# image = cv2.line(img, tuple(imgpts[3].ravel()), tuple(imgpts[2].ravel()), (0,0,255), 5)
			
			cv2.circle(img,self.center,4,(255,0,0),10)
			if self.center[0] < 320 + 5 and self.center[0] > 320 - 5:
				print "offset :", x_offset
				cv2.circle(img,(320,240),4,(0,255,0),20)
			else:
				cv2.circle(img,(320,240),4,(0,0,255),10)
			# if (self.center[0] == 320):
			# 	cv2.circle(img,(320,240),4,(0,255,0),20)
			# 	print "CENTERED"
			# else:
			# 	cv2.circle(img,(320,240),4,(0,0,255),10)
			cv2.imshow('img', img)
			k = cv2.waitKey(1) & 0xff

			# For displaying drawMatches, which shows the matching points in real time
			# self.drawMatches(self.target_image, self.kp, gray, kp, good)

		else:
			print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
			matchesMask = None
			rect = None
			imgpts = None
			# imgpts2 = None
			# rect = np.zeros((4, 1, 2), dtype=np.int)
			# imgpts = np.zeros((3, 1, 2), dtype=np.int)
			# imgpts2 = imgpts
		
	# From https://stackoverflow.com/questions/20259025/module-object-has-no-attribute-drawmatches-opencv-python
	def drawMatches(self, img1, kp1, img2, kp2, matches):
	# """
	# My own implementation of cv2.drawMatches as OpenCV 2.4.9
	# does not have this function available but it's supported in
	# OpenCV 3.0.0

	# This function takes in two images with their associated 
	# keypoints, as well as a list of DMatch data structure (matches) 
	# that contains which keypoints matched in which images.

	# An image will be produced where a montage is shown with
	# the first image followed by the second image beside it.

	# Keypoints are delineated with circles, while lines are connected
	# between matching keypoints.

	# img1,img2 - Grayscale images
	# kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint 
	# 		  detection algorithms
	# matches - A list of matches of corresponding keypoints through any
	# 		  OpenCV keypoint matching algorithm
	# """

		# Create a new output image that concatenates the two images together
		# (a.k.a) a montage
		rows1 = img1.shape[0]
		cols1 = img1.shape[1]
		rows2 = img2.shape[0]
		cols2 = img2.shape[1]

		# Create the output image
		# The rows of the output are the largest between the two images
		# and the columns are simply the sum of the two together
		# The intent is to make this a colour image, so make this 3 channels
		out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

		# Place the first image to the left
		out[:rows1,:cols1] = np.dstack([img1, img1, img1])

		# Place the next image to the right of it
		out[:rows2,cols1:] = np.dstack([img2, img2, img2])

		# For each pair of points we have between both images
		# draw circles, then connect a line between them
		for mat in matches:

			# Get the matching keypoints for each of the images
			img1_idx = mat.queryIdx
			img2_idx = mat.trainIdx

			# x - columns
			# y - rows
			(x1,y1) = kp1[img1_idx].pt
			(x2,y2) = kp2[img2_idx].pt

			# Draw a small circle at both co-ordinates
			# radius 4
			# colour blue
			# thickness = 1
			cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)   
			cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)

			# Draw a line in between the two points
			# thickness = 1
			# colour blue
			cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255,0,0), 1)



		# Show the image
		cv2.imshow('Matched Features', out)
		k = cv2.waitKey(1) & 0xff
		# cv2.waitKey(0)
		# cv2.destroyWindow('Matched Features')

		# Also return the image if you'd like a copy
		return out


if __name__ == "__main__":
	rospy.init_node('homography')
	homography = Homography()
rospy.spin()
