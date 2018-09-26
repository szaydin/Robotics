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

class CancelGoal(State):
  def  __init__(self):
    smach.State.__init__(self, outcomes=['cancel_outcome'])
    self.cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)

  def execute(self, userdata):
     setCancelState(True)
     goalId = GoalID()
     self.cancel_pub.publish(goalId)    
     return 'cancel_outcome'