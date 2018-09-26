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

# Location of marker
marker_orientation = (0,0,0.791087389804, 0.611703148343)
marker_waypoint = [[(1.65094284374,2.68850808758,0),marker_orientation]]

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

# Navigation moves the turtlebot closer to the client's location.
# Hard-coded at the moment.
class Navigation(State):
	def __init__(self):
		State.__init__(self, outcomes=['done'])
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		rospy.loginfo("waiting for move_base server...")
		self.client.wait_for_server()
		rospy.loginfo("move_base server found")

	def execute(self,userdata):
		rospy.loginfo("moving to client")
		self.client.send_goal(goal_pose(marker_waypoint[0]))
		self.client.wait_for_result()
		rospy.loginfo("at client")

		time.sleep(5)
		return 'done'