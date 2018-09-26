import rospy
import os
import time
import math
import actionlib
import numpy as np
import argparse
import imutils
import glob
from smach import State, StateMachine
from smach_ros import SimpleActionState
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class NavSound(State):
	def __init__(self):
		State.__init__(self, outcomes = ['done'])
		self.directory = os.path.dirname(os.path.abspath(__file__))
		self.soundSource = self.directory + '/arrival.wav'
		self.soundHandle = SoundClient()
	def execute(self, userdata):
		self.soundHandle.playWave(self.soundSource)
		return 'done'