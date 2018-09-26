import rospy
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
import json
import urllib2
import time #for sleep()
import roslib
from kobuki_msgs.msg import PowerSystemEvent, AutoDockingAction, AutoDockingGoal, SensorState #for kobuki base power and auto docking
from kobuki_msgs.msg import ButtonEvent #for kobuki base's b0 button
from smart_battery_msgs.msg import SmartBatteryStatus #for netbook battery
import math #for comparing if Kobuki's power has changed using fabs



class Docking(State):
	def __init__(self):
		State.__init__(self, outcomes=['docking_complete', 'docking_failed'])
		self._client = actionlib.SimpleActionClient('/dock_drive_action', AutoDockingAction)
		rospy.loginfo("waiting for auto_docking server...")
		self._client.wait_for_server()
		rospy.loginfo("auto_docking server found")

	def execute(self, userdata):
		goal = AutoDockingGoal()
		rospy.loginfo("Sending auto_docking goal and waiting for result (times out in 180 seconds and will try again if required)")
		self._client.send_goal(goal)
		success = self._client.wait_for_result(rospy.Duration(60))
		if success:
			rospy.loginfo("Auto_docking successful")
			# The callback which detects the docking status can take up to 3 seconds to update which was
			# causing the robbot to try and redock (presuming it failed) even when the dock was successful.  
			# Therefore hardcoding this value after success.
			self.charging_at_dock_station = True 
			success = True
			return 'docking_complete'
		else:
			rospy.loginfo("Auto_docking failed")
			success = False
			return 'docking_failed'	