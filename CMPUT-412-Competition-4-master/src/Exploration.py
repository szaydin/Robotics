#!/usr/bin/env python
import rospy
import os
import time
import math
import actionlib
import numpy as np
import argparse
import imutils
import glob
import glob
import cv2, cv_bridge
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from numpy import cross, eye, dot
from scipy.linalg import expm3, norm
from matplotlib import pyplot as plt
from visualization_msgs.msg import Marker

MIN_MATCH_COUNT = 25

# # Orientation Variables
# cone1 = (0.0, 0.0, -0.908091001459, 0.418772889606)
# cone2 = (0.0, 0.0, -0.28695816815,  0.957943114038)
# cone3 = (0.0, 0.0, 0.492298195017,  0.870426612175)
# cone4 = (0.0, 0.0, 0.970629644687, 0.240578662509)
# finish_orientation = (0.0, 0.0,-0.704462374067,0.709741335646)

# # Expected
# # A list of waypoints for the robot to navigate around
# waypoints = [[(0.135687176908, 2.87663573268, 0.0 ) , cone1],
# 		[(0.180769769101, 0.444637976371, 0.0), cone2],
# 		[(6.04934212359, 1.27606519063, 0.0), cone3],
# 		[( 5.29041238011, 3.86765393351, 0.0), cone4]]

# finish = [[(-0.239126570389, 1.84109961989, 0.0) , finish_orientation]]

# Orientation Variables
# cone1 = (0.0, 0.0, -0.702107066585, 0.712071391822)
# cone2 = (0.0, 0.0, 0.0992879822038,  0.995058740271)
# cone3 = (0.0, 0.0, 0.762208617238,  0.647331463632)
# cone4 = (0.0, 0.0, -0.99480092466, 0.101838697438)
# finish_orientation = (0.0, 0.0,-0.704462374067,0.709741335646)

# # Expected
# # A list of waypoints for the robot to navigate around
# waypoints = [[(-1.04513289827, 2.65288310459, 0.0 ) , cone1],
# 		[(-0.906756684211, -0.610367251425, 0.0), cone2],
# 		[(8.06608547521, 1.08283178104, 0.0), cone3],
# 		[( 6.50486606394, 4.0904250122, 0.0), cone4]]

cone1 = (0.0, 0.0, -0.737724583204, 0.67510179924)
cone2 = (0.0, 0.0, -0.00870532085858,  0.999962107976)
cone3 = (0.0, 0.0, 0.806638644695,  0.591044919515)
cone4 = (0.0, 0.0, -0.98566574034, 0.168709953234)
finish_orientation = (0.0, 0.0,-0.704462374067,0.709741335646)

# Expected
# A list of waypoints for the robot to navigate around
waypoints = [[(-1.47449619273, 3.12128817822, 0.0 ) , cone1],
		[(-1.21680523622, -0.91390214102, 0.0), cone2],
		[(8.71551054151, 1.36531343529, 0.0), cone3],
		[( 6.58077411223, 4.74505223577, 0.0), cone4]]
finish = [[(-0.239126570389, 1.84109961989, 0.0) , finish_orientation]]

def goal_pose(pose):
	goal_pose = MoveBaseGoal()
	goal_pose.target_pose.header.frame_id = 'map'
	goal_pose.target_pose.pose.position.x = pose[0][0]
	goal_pose.target_pose.pose.position.y = pose[0][1]
	goal_pose.target_pose.pose.position.z = pose[0][2]
	goal_pose.target_pose.pose.orientation.x = pose[1][0]
	goal_pose.target_pose.pose.orientation.y = pose[1][1]
	goal_pose.target_pose.pose.orientation.z = pose[1][2]
	goal_pose.target_pose.pose.orientation.w = pose[1][3]
	return goal_pose

def getTimeSafe():
	while True:
		# rospy may returns zero, so we loop until get a non-zero value.
		time = rospy.Time.now()
		if time != rospy.Time(0):
			return time

class Exploration(State):
	def __init__(self):
		State.__init__(self, outcomes=['markerFound', 'done'])
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.client.wait_for_server()

		self.markerFound = False

		self.directory = os.path.dirname(os.path.abspath(__file__))
		# self.soundSource = self.directory + '/wilhelm.wav'
		self.soundSource = self.directory + '/yay.wav'
		self.soundSource2 = self.directory + '/wilhelm.wav'
		print self.soundSource
		self.soundHandle = SoundClient()
		self.currentPosition = None

	def execute(self, userdata):
		print('moving to first point')
		homography = Homography()
		ARTrack = Track()

		if ARTrack == 'found' or homography == 'found':
			self.client.cancel_all_goals()
			self.client.cancel_goal()
			return 'markerFound'
		# Start by going to the first cone/waypoint
		self.client.send_goal(goal_pose(waypoints[0]))

		if Homography() == 'found':
			self.client.cancel_all_goals()
			self.client.cancel_goal()
			self.soundHandle.playWave(self.soundSource2)
			return 'markerFound'
		self.client.wait_for_result()

		self.client.send_goal(goal_pose(waypoints[1]))
		if Track() == 'found':
			self.client.cancel_all_goals()
			self.client.cancel_goal()
			self.soundHandle.playWave(self.soundSource)
			return 'markerFound'
		if Homography() == 'found':
			self.client.cancel_all_goals()
			self.client.cancel_goal()
			self.soundHandle.playWave(self.soundSource2)
			return 'markerFound'

		self.client.wait_for_result()

		self.client.send_goal(goal_pose(waypoints[2]))

		if Track() == 'found':
			self.client.cancel_all_goals()
			self.client.cancel_goal()
			self.soundHandle.playWave(self.soundSource)
			return 'markerFound'
		if Homography() == 'found':
			self.client.cancel_all_goals()
			self.client.cancel_goal()
			self.soundHandle.playWave(self.soundSource2)
			return 'markerFound'
		self.client.wait_for_result()
		self.client.send_goal(goal_pose(waypoints[3]))

		if Track() == 'found':
			self.client.cancel_all_goals()
			self.client.cancel_goal()
			self.soundHandle.playWave(self.soundSource)
			return 'markerFound'
		if Homography() == 'found':
			self.client.cancel_all_goals()
			self.client.cancel_goal()
			self.soundHandle.playWave(self.soundSource2)
			return 'markerFound'

		self.client.wait_for_result()

		print('at first point')
		# self.soundHandle.playWave(self.soundSource)

		# for pose in waypoints:
		# 	goal = goal_pose(pose)
		# 	self.client.send_goal(goal)
		# 	if Track() == 'found':
		# 		self.client.cancelGoal()
		# 		self.soundHandle.playWave(self.soundSource)
		# 	self.client.wait_for_results()
		# # print "done one lap."
		return 'done'

class Homography:
	def __init__(self):
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.bridge = cv_bridge.CvBridge()
		self.cam_info_sub = rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self.info_cb)
		self.img_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.img_cb)
		self.directory = os.path.dirname(os.path.abspath(__file__))
		self.soundSource = self.directory + '/yay.wav'
		print self.soundSource
		self.soundHandle = SoundClient()
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
				print "CENTERED"
				cv2.circle(img,(320,240),4,(0,255,0),20)
				self.soundHandle.playWave(self.soundSource)
				self.client.cancel_goal()
				self.client.cancel_all_goals()


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
	

class Track:
	def __init__(self):
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.bridge = cv_bridge.CvBridge()
		self.cam_info_sub = rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self.info_cb)
		self.img_sub = rospy.Subscriber( '/usb_cam/image_raw', Image, self.img_cb)
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
											Twist, queue_size=1)
		self.twist = Twist()
		# print "something"
		self.marker = rospy.Subscriber('visualization_marker', Marker, self.marker_cb)
		self.axis = np.float32([[0.1,0,0], [0,0.1,0], [0,0,-0.1],[0,0,0]]).reshape(-1,3)
		self.directory = os.path.dirname(os.path.abspath(__file__))
		self.soundSource = self.directory + '/wilhelm.wav'
		self.soundHandle = SoundClient()
		self.rvecs = None
		self.tvecs = None
		self.D = None
		self.K = None
		#self.imgpts = None
		self.img = None
		self.center = 320
	def info_cb(self, msg):
		self.K = np.array(msg.K).reshape(3,3)
		self.D = np.array(msg.D)

	def img_cb(self, msg):
		self.img  = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		self.img  = self.img[50:500]
		# cv2.imshow('image',self.img)
		# cv2.waitKey(1)

	def marker_cb(self, msg):
		self.soundHandle.playWave(self.soundSource)
		self.client.cancel_goal()
		self.client.cancel_all_goals()
		return 'found'
		
		print ("msg.id = ", msg.id)
		if (msg.id != 0):
			px = msg.pose.position.x
			py = msg.pose.position.y
			pz = msg.pose.position.z
			ox = msg.pose.orientation.x
			oy = msg.pose.orientation.y
			oz = msg.pose.orientation.z
			ow = msg.pose.orientation.w

			tvecs = np.array([px, py, pz])

			angle = 2 * math.acos(ow)
			x = ox / math.sqrt(1 - ow*ow)
			y = oy / math.sqrt(1 - ow*ow)
			z = oz / math.sqrt(1 - ow*ow)
			ratio = math.sqrt(x*x + y*y + z*z)
			print x

			#normalize them
			x = x / ratio*angle
			y = y / ratio*angle
			z = z / ratio*angle
			rvecs = np.array([x, y, z])
			# print "rvecs: ", rvecs

			#rvecs = msg.getAngle().getAxis() bullet
			# print(rvecs)

			imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.K, self.D)
			if imgpts[3][0][0] < self.center + 2 and imgpts[3][0][0] > self.center - 2:
				print "CENTERED"
				self.soundHandle.playWave(self.soundSource)
			self.soundHandle.playWave(self.soundSource)

			image = cv2.line(self.img, tuple(imgpts[3].ravel()), tuple(imgpts[0].ravel()), (255,0,0), 5)
			image = cv2.line(self.img, tuple(imgpts[3].ravel()), tuple(imgpts[1].ravel()), (0,255,0), 5)
			image = cv2.line(self.img, tuple(imgpts[3].ravel()), tuple(imgpts[2].ravel()), (0,0,255), 5)
		cv2.imshow('img',self.img)
		k = cv2.waitKey(1) & 0xff
