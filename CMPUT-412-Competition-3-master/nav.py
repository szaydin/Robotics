#!/usr/bin/env python

import rospy
import actionlib
import tf
import os

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy

client = None
sleep = False
start = False

# Orientation Variables
cone1 = (0.0, 0.0, -0.908091001459, 0.418772889606)
cone2 = (0.0, 0.0, -0.28695816815,  0.957943114038)
cone3 = (0.0, 0.0, 0.492298195017,  0.870426612175)
cone4 = (0.0, 0.0, 0.970629644687, 0.240578662509)
finish_orientation = (0.0, 0.0,-0.704462374067,0.709741335646)
# Expected
# A list of waypoints for the robot to navigate around
# waypoints = [[(0.135687176908, 2.87663573268, 0.0 ) , cone1],
# 		[(0.180769769101, 0.444637976371, 0.0), cone2],
# 		[(6.04934212359, 1.27606519063, 0.0), cone3],
# 		[( 5.29041238011, 3.86765393351, 0.0), cone4]]

waypoints = [[(0.135687176908, 2.87663573268, 0.0 ) , cone1],
		[(0.150769769101, 0.1, 0.0), cone2],
		[(6.5934212359, 0.85, 0.0), cone3],
		[( 5.5041238011, 4.05ww, 0.0), cone4]]

finish = [[(-0.239126570389, 1.84109961989, 0.0) , finish_orientation]]



# A helper function to turn a waypoint into a 'MoveBaseGoal'
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

def sleep_switch(msg):
	global client
	global sleep
	global start

	if msg.buttons[2] == 1:
		sleep = not sleep

	elif msg.buttons[1] == 1:
		print("Stopping.")
		start = (start == False)
		client.cancel_goal()
		os.exit(0)

if __name__== '__main__':

	rospy.init_node('nav')
	joy_s = rospy.Subscriber('joy', Joy, sleep_switch)
	laps = 0

	client = actionlib.SimpleActionClient('move_base', MoveBaseAction) # Create a simple action client, and wait for the server to be ready
	client.wait_for_server()

	# # Start by going to the first cone/waypoint
	# client.send_goal(goal_pose(waypoints[0]))
	# client.wait_for_result()
	# print('at first point')

	# client.send_goal(goal_pose(waypoints[1]))
	# client.wait_for_result()
	# print('at second point')

	# client.send_goal(goal_pose(waypoints[2]))
	# client.wait_for_result()
	# print('at third point')

	# client.send_goal(goal_pose(waypoints[3]))
	# client.wait_for_result()
	# print('at fourth point')

	# # End
	# client.send_goal(goal_pose(waypoints[0]))
	# client.wait_for_result()
	# print('at first point')
	print('moving to first point')
	# Start by going to the first cone/waypoint
	client.send_goal(goal_pose(waypoints[0]))
	client.wait_for_result()
	print('at first point')

	print 'ready'
	while not sleep:
		pass

	while laps < 2:
		# Loop through the waypoints, sending each as an action goal
		for pose in waypoints:
			goal = goal_pose(pose)
			client.send_goal(goal)
			client.wait_for_result()
		laps = laps + 1
		print("laps: ", laps)

	client.send_goal(goal_pose(waypoints[0]))
	client.wait_for_result()

	client.send_goal(goal_pose(finish[0]))
	client.wait_for_result()
	print('finish')