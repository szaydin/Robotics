import rospy
import smach
import smach_ros
import math
import random
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from kobuki_msgs.msg import AutoDockingAction,AutoDockingGoal,PowerSystemEvent,SensorState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Twist

class Undock(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success'])
		self.sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
		self.cmd_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist,
			queue_size=2)
		self.pose = Pose()

	def odom_cb(self, msg):
		self.pose = msg.pose.pose

	def execute(self, userdata):
		initial_pose = self.pose
		d = 0
		# back up for 0.5 meters to get away from dock
		while d < 0.5 and not rospy.is_shutdown():
			cmd = Twist()
			cmd.linear.x = -0.2
			self.cmd_pub.publish(cmd)
			rospy.sleep(0.1)
			pose = self.pose
			dx = self.pose.position.x - initial_pose.position.x
			dy = self.pose.position.y - initial_pose.position.y
			d = math.hypot(dx, dy)
		# send a stop command for good measure
		cmd = Twist()
		self.cmd_pub.publish(cmd)
		rospy.loginfo("undocked.")
		return 'success'