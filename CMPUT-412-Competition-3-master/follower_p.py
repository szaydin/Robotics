#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image, Joy
from geometry_msgs.msg import Twist

sleep_bot = True
		

def sleep_switch(msg):
	global sleep_bot
	if msg.buttons[2] == 1:
		print("X Button was pressed.")
		sleep_bot = (sleep_bot == False)

def got_twist(msg):
	global g_twist
	g_twist = msg



class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("image_window", 2)
		cv2.namedWindow("maskLEFT_window", 1)
		cv2.namedWindow("maskRIGHT_window",1)

		joy_sub = rospy.Subscriber('joy', Joy, sleep_switch)
		cmd_vel_with_dead_bot_switch = rospy.Subscriber('cmd_vel_safe', Twist, got_twist)

		# cv2.namedWindow("window2", 1)
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
											Image, self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
											Twist, queue_size=1)

		# self.joy_sub = rospy.Subscriber('joy', Joy, self.sleep_switch)
		# self.cmd_vel_with_dead_bot_switch = rospy.Subscriber('cmd_vel_safe', Twist, self.got_twist)
		self.twist = Twist()
		self.threshold = 15
		self.sleep_bot = True
		self.prev_err = 0
		self.prev_time = 0
		self.dampening = 0
		self.curr_time = 0
		self.delta_time = self.curr_time - self.prev_time
		self.delta_err = 0
		self.turn_angle = 0
		self.find_lane = True
		self.err = 0
		self.dif = 0

		self.left_lock = False
		self.right_lock = False
		self.right_turn = False
		self.left_turn = False

	# def sleep_switch(msg):
	# 	if msg.buttons[2] == 1:
	# 		print("X Button was pressed.")
	# 		self.sleep_bot = (self.sleep_bot == False)

	# def got_twist(msg):
	# 	self.g_twist = msg

	def image_callback(self, msg):
		global sleep_bot
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
		blur_image = cv2.GaussianBlur(gray_image, (5,5) ,0)
		lower_white = numpy.array([ 0,  50,  170])
		upper_white = numpy.array([10, 180, 255])
		lower_white1 = numpy.array([ 170,  50,  170])
		upper_white1 = numpy.array([255, 180, 255])
		# mask = cv2.inRange(hsv, lower_white, upper_white)
		mask = cv2.inRange(hsv, lower_white1, upper_white1)

	  
		h, w, d = image.shape
		search_top = 3*h/8
		search_bot = h

		# Masking so that we only get a square of vision
		# Left Side Masking
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0

		# For the left side mask (mask1) we display a green circle on lockon
		M = cv2.moments(mask)
		#M2 = cv2.moments(mask2)

		if sleep_bot != True:

			if M['m00'] > 0:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				
				cv2.circle(image, (cx, cy), 15, (0,255,0), -1)
				

			if M['m00'] > 0 and self.find_lane == True> 0:
				print("Cone found.")
				self.right_lock = True
				self.left_lock = True


				self.err = cx - w/2
				self.dif = self.err - self.dif


				# BEGIN CONTROL

				self.twist.linear.x = 0.2
				self.twist.angular.z =(-float(self.err) / 100) 
				self.prev_err = self.err
				self.prev_time = self.curr_time
			
				cv2.circle(image, (center_x, cy), 15, (255,0,0), -1)


		elif sleep_bot == True:
			print("Sleeping...")


		cv2.imshow("maskLEFT_window", mask)
		cv2.imshow("image_window", image)
		cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
