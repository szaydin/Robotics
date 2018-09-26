#!/usr/bin/env python
import numpy as np
import cv2
import time
import rospy
from sensor_msgs.msg import Joy
from smach import State, StateMachine

g_is_dead_bot = True

def dead_bot_switch(msg):
	global g_is_dead_bot
	print("key is", msg.buttons[2])
	if msg.buttons[2]==1 :
		print ("X Button was pressed")
		g_is_dead_bot = (g_is_dead_bot==False)
	if msg.buttons[2]==0 :
		g_is_dead_bot = (g_is_dead_bot==True)

	print(g_is_dead_bot)

class TakePicture(State):
	def __init__(self):
		State.__init__(self, outcomes=['done'])
		self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_alt2.xml')
		self.eye_cascade =  cv2.CascadeClassifier('haarcascade_eye.xml')
		g_is_dead_bot = True
		joy_sub = rospy.Subscriber('joy', Joy, dead_bot_switch)
		# rospy.init_node('face_det')

	def execute(self, userdata):
		global g_is_dead_bot
		
		cap = cv2.VideoCapture(1)
		print 'something'
		detect = True

		while True:
			ret, img = cap.read()
			save_im = img.copy()
			if (g_is_dead_bot != True): #wait for 's' to be pressed to save the image into new file 
				print("new switch is", g_is_dead_bot)
				cv2.imwrite('messigray.png',save_im)
				cv2.destroyAllWindows()
				return 'done'

			gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			faces = self.face_cascade.detectMultiScale(gray, 1.3, 5) #detecting the face by using detectMultiScale
			for (x,y,w,h) in faces: 
				cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2) #drawing rectangle on face based on x,y,w,h values	
					#roi_gray = gray[y:y+h, x:x+w] #masking gray image to search for eyes later
					#roi_color = img[y:y+h, x:x+w] #masking original image to search for eyes later
				#eyes = eye_cascade.detectMultiScale(roi_gray) #detecting eyes by using MultiScale
					#for (ex,ey,ew,eh) in eyes:
						#	cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2) #drawing rectangle on eyes based on ex,ey,ew and eh
				cv2.imshow('img',img)

				k = cv2.waitKey(30) & 0xff
				if k == 27:
					cv2.destroyAllWindows()

		cap.release()