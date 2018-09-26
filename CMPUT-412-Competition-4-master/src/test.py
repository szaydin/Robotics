#!/usr/bin/env python
import numpy as np
import argparse
import imutils
import glob
import cv2
import time

threshold = 0.34

def detect():
	cap = cv2.VideoCapture(1)
	#time.sleep(3)
	template = cv2.imread("ar.png", 0)
	#print("template is",template)
	#template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
	template = cv2.Canny(template, 70, 250, 5)
	#(tH, tW) = template.shape[:2]
	#print( tH, tW)
	#cv2.imshow("Template", template) 
	templateGray = cv2.resize(template, (0,0), fx=0.5, fy=0.5) 
	while True:
		ret, img = cap.read()
		img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
		#img = imutils.resize(img, width = int(img.shape[1] * 0.5))
		imageGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		#templateGray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
		result = cv2.matchTemplate(imageGray,templateGray, cv2.TM_CCORR_NORMED)
		min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
		top_left = max_loc
		print(max_val)
		h,w = templateGray.shape
		bottom_right = (top_left[0] + w, top_left[1] + h)
		if max_val > threshold:
			cv2.rectangle(img,top_left, bottom_right,(0,0,255),4)
	
		cv2.imshow("Template", template)
		cv2.imshow("Result", img)
		if cv2.waitKey(1) & 0xFF == ord('q'):
        		break
	cap.release()
	cv2.destroyAllWindows()
detect()
