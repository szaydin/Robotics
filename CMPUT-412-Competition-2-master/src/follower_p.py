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
		self.threshold = 70 # Increase value in more brightly lit rooms, decrease in darker rooms.
		self.sleep_bot = True
		self.prev_err = 0
		self.prev_time = 0
		self.dampening = 0
		self.curr_time = 0
		self.delta_time = self.curr_time - self.prev_time
		self.delta_err = 0
		self.turn_angle = 0
		self.speed = 0.8

		self.err = 0

		self.left_lock = False
		self.right_lock = False
		self.right_turn = False
		self.left_turn = False
		self.compensation = 5.7

	def image_callback(self, msg):
		global sleep_bot
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_white = numpy.array([ 0,  0,  255-self.threshold])
		upper_white = numpy.array([255, self.threshold, 255])
		mask = cv2.inRange(hsv, lower_white, upper_white)
		mask2 = cv2.inRange(hsv, lower_white, upper_white)
	    
		h, w, d = image.shape
		search_top = 3*h/5 - 30
		# search_bot = 3*h/4
		search_bot = h

		search_left_mask1 = w-(3*w/4) - 150 
		search_right_mask1 = w-(3*w/4) - 20

		search_right_mask2 = 3*w/4 + 200
		search_left_mask2 = 3*w/4 + 20

		# Masking so that we only get a square of vision
		# Left Side Masking
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0
		mask[0:h, 0:search_left_mask1] = 0
		mask[0:h, search_right_mask1:w] = 0

		# Right Side Masking
		mask2[0:search_top, 0:w] = 0
		mask2[search_bot:h, 0:w] = 0
		mask2[0:h, search_right_mask2:w] = 0
		mask2[0:h, 0:search_left_mask2] = 0

		# For the left side mask (mask1) we display a green circle on lockon
		M = cv2.moments(mask)
		M2 = cv2.moments(mask2)

		if sleep_bot != True:

			if M['m00'] > 0:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				cv2.circle(image, (cx, cy), 15, (0,255,0), -1)

		    # For the right side mask (mask2) we display a red circle on lockon
			if M2['m00'] > 0:
				cx2 = int(M2['m10']/M2['m00'])
				cy2 = int(M2['m01']/M2['m00'])
				cv2.circle(image, (cx2, cy2), 15, (0,0,255), -1)

			# We most likely lost the left lane, so its most likely a sharp
			# left turn. To compensate, we increase the dampening
			# by further lowering dampening.
			if M['m00'] <= 0:
				print("Lost left lane. Compensating..")
				self.dampening = self.dampening + self.compensation
				# self.twist.linear.x = (self.speed - abs(self.dampening) / 50)
				self.twist.angular.z = (-float(5*self.err) + self.dampening) / 60
				self.cmd_vel_pub.publish(self.twist)

			# We most likely lost the right lane, so its most likely a sharp
			# right turn. To compensate, we increase the dampening
			# by further lowering dampening.
			if M2['m00'] <= 0:
				print("Lost right lane. Compensating..")
				self.dampening = self.dampening - self.compensation
				# self.twist.linear.x = (self.speed - abs(self.dampening) / 50)
				self.twist.angular.z = (-float(5*self.err) + self.dampening) / 60
				self.cmd_vel_pub.publish(self.twist)


			if M['m00'] > 0 and M2['m00'] > 0:
				print("Lane found.")
				self.right_lock = True
				self.left_lock = True
				center_x = (cx + cx2)/2
				center_y = (cy + cy2)/2
				self.turn_angle = math.atan2(cy2-cy, cx2-cx)
				self.curr_time = rospy.get_time()
				self.delta_err = self.err - self.prev_err
				self.dampening = -(float(19*self.turn_angle))
				cv2.circle(image, (center_x, center_y), 15, (255,0,0), -1)

				self.err = center_x - w/2

				# if the camera is level
				if self.err == 0:
					cv2.circle(image, (center_x, center_y), 15, (255,0,0), -1)

				elif self.err != 0:
					cv2.circle(image, (center_x, center_y), 15, (0,255,255), -1)

				print("turn angle ",self.turn_angle)
				print("delta err", self.delta_err)
				print("delta_time ", self.delta_time)
				print("dampening", self.dampening)
				print("err ", self.err)

				self.twist.linear.x = (self.speed - abs(self.dampening) / 25)
				self.twist.angular.z = (-float(5*self.err) + self.dampening) / 125
				self.prev_err = self.err
				self.prev_time = self.curr_time
				
				self.cmd_vel_pub.publish(self.twist)

		# elif sleep_bot == True:
		# 	print("Sleeping...")


		cv2.imshow("maskLEFT_window", mask)
		cv2.imshow("maskRIGHT_window", mask2)
		cv2.imshow("image_window", image)
		cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
