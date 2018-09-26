#!/usr/bin/env python

"""
Concept from https://github.com/BrianEverRoboticsTeam/Evasion-Pursuit-Competition
"""

import rospy
from sensor_msgs.msg import Image, Joy
from geometry_msgs.msg import Twist
from movement import Movement
import cv2
import numpy as np
from cv_bridge import CvBridge
import math


sleep_bot = True # We start off the robot as 'sleeping' so we start only when we press the 'X' button on the logitech controller
# Return a new array of given shape and data-type (uint8 = Unsigned integer from 0 to 255 [since 8 bit]), filled with zeros
# 'frame' is the input image
frame = np.zeros([480, 640, 3], dtype=np.uint8) 
depth_image = None
display = True

# Concept from http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
bridge = CvBridge()
cascade = cv2.CascadeClassifier('kobuki_haar.xml')

# Puts robot to sleep when the 'X' button on the logitech controller
# is pressed, and wakes up when it is pressed again
def sleep_switch(msg):
    global sleep_bot

    if msg.buttons[2]==1:
        print ("X Button was pressed")
        display = False
        sleep_bot = (sleep_bot == False)
        
def got_twist(msg):
    global g_twist
    g_twist = msg

def image_callback(msg):
    global frame
    try:
    	# Converting ROS image to OpenCV image
    	# Note that the 'frame' is takes the converted values
        tmp = bridge.imgmsg_to_cv2(msg) 
        frame[..., 0] = tmp[..., 2]
        frame[..., 1] = tmp[..., 1]
        frame[..., 2] = tmp[..., 0]
    except:
        frame = None

def depth_image_callback(msg):
    global depth_image
    depth_image = bridge.imgmsg_to_cv2(msg)

    
if __name__ == '__main__':

    rospy.init_node('follower')
    get_image = rospy.Subscriber('camera/rgb/image_raw', Image, image_callback)
    get_depth_image = rospy.Subscriber('/camera/depth/image', Image, depth_image_callback)

    # For Logitech Controller Start.
    joy_sub = rospy.Subscriber('joy', Joy, sleep_switch)
    cmd_vel_with_dead_bot_switch = rospy.Subscriber('cmd_vel_safe', Twist, got_twist)

    cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    # Parameters for movement
    move = Movement()
    twist = Twist()
    rate = rospy.Rate(10)
    decay = 0.15
    # exist_last_tw = False
    target_locked = False

    # Parameters for optical sensors/computer vision (cv)
    prev_frame_gray = None
    corners = None
    mask = None
    prev_box = None
    box = None
    usebox = False

    # Parameters for current motion and current location tracking
    destination_x = None
    destination_y = None
    previous_x = None
    previous_y = None

    

    # To track Optical Flow (the pattern of apparent motion of images between two consecutive frames)
    # Parameters for ShiTomasi corner detection
    # Concept from https://docs.opencv.org/3.3.1/d7/d8b/tutorial_py_lucas_kanade.html
    feature_params = dict( maxCorners = 100,
            qualityLevel = 0.3,
            minDistance = 7,
            blockSize = 7 )

    # Parameters for Lucas Kanade Optical Flow
    lk_params = dict( winSize  = (15,15),
            maxLevel = 2,
            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    # The main while loop which runs as long as the program itself is shutdown
    while not rospy.is_shutdown():

        # Checks if the sleep_bot flag is True (if the 'X' button was pressed on the logitech controller)
        # part of competition requirements
        if (sleep_bot != True):
            if frame != None:
                frame_bkp = frame.copy()

                # We convert the input image 'frame' to gray scale 'cv2. COLOR_BGR2GRAY'
                # We convert a BGR image to Grayscale to get other flags
                # Concept from http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Detects objects of different sizes in the input image 'frame'. The detected images are returned as a list of rectangles
                # Concept from https://docs.opencv.org/2.4/modules/objdetect/doc/cascade_classification.html
                loc = cascade.detectMultiScale(frame, 1.3, 5)

                for box in loc:

                    if prev_box != None and target_locked:
                        check_dist = 0.0
                        for i in range(4):
                            check_dist += (box[i] - prev_box[i]) ** 2
                        check_dist = math.sqrt(check_dist)
                        if check_dist > 50:
                            # Target is out of range, so we can ignore it.
                            continue

                    if prev_box != None:
                        for i in range(4):
                            box[i] = int(round(prev_box[i] * decay + box[i] * (1 - decay) ))

                    x, y, w, h = box

                    # For displaying the lock-on on screen
                    mid_x = x + w / 2
                    mid_y = y + h / 2
                    
                    destination_x = mid_x
                    destination_y = mid_y

                    if mid_y < 100:
                        continue

                    if display:
                        cv2.rectangle(frame_bkp, (x, y), (x+w, y+h), (255, 0, 0), 2)

                    mask = np.zeros_like(frame_gray, dtype=np.uint8)
                    mask[y:y+h, x:x+w] = 1

                    prev_box = box
                    usebox = True
                    target_locked = True
                    break

                # Concept from https://docs.opencv.org/3.0-beta/modules/imgproc/doc/feature_detection.html
                if mask != None:
                    corners = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
                    mask = None
                    print 'new points generated'

                # Note: cv2.calcOpticalFlowPyrLK calculates an optical flow for a sparse feature set
                #       using the iterative Lucas-Kanade method with pyramids
                # Concept from https://docs.opencv.org/2.4/modules/video/doc/motion_analysis_and_object_tracking.html
                elif corners != None and len(corners.shape) == 3:
                    if prev_frame_gray != None:
                    	# The error is the distance between the center of the fram (center of the screen being displayed)
                    	# and the center of the locked-target's lock on point (center of the circle)
                        corners, st, err = cv2.calcOpticalFlowPyrLK(prev_frame_gray, frame_gray, corners, None, **lk_params)

                        npt = np.sum(st==1)
                        print '%d points tracked'%(npt)
                        if npt == 0:
                            print 'lost all points'
                            corners = None
                        else:
                            corners = corners[st==1]
                            corners = corners[:, np.newaxis, :]

                            destination_x = corners[:, 0, 0].mean().astype(np.int32)
                            destination_y = corners[:, 0, 1].mean().astype(np.int32)

                # Only start only when we are locked on-to target
                if (destination_y != None and destination_x != None and depth_image != None):
                    if (previous_x != None and previous_y != None):

                        destination_x = decay * previous_x + (1-decay) * destination_x
                        destination_y = decay * previous_y + (1-decay) * destination_y

                    # Begin motion if locked on to target
                    move.start()
                    x_offset = destination_x - 320
                    if abs(x_offset) > 10:
                        twist.angular.z = - x_offset * 0.01
                    else:
                        twist.angular.z = 0

                    # The distance is the average of the array elements
                    dist = np.nanmean(depth_image[destination_y-5:destination_y+5, destination_x-5:destination_x+5])

                    # if the distance is Not applicable (too close), so set it to 0.8 so it backs up
                    # until range is sufficient. Every loop, this condition is checked so we keep
                    # backing up if other robot is too close
                    if math.isnan(dist):
                        dist = 0.8

                    dist_offset = dist - 0.8
                    twist.linear.x = dist_offset * 1.5

                    if twist.linear.x > 0.9:
                        twist.linear.x = 0.9

                    move.updateTarget(twist)
                    cmd_vel_pub.publish(move.step())

                    # exist_last_tw = True

                    if display:
                        ix = int(round(destination_x))
                        iy = int(round(destination_y))

                        # Display green circle on locked on point
                        cv2.circle(frame_bkp, (ix, iy), 5, (124,252,0), -1)

                    previous_x = destination_x
                    previous_y = destination_y
                    destination_x = None
                    destination_y = None

                else:
                    print 'Target Lost!'
                    target_locked = False

                if display and corners != None:
                    for i in corners.astype(np.int32):
                        cx, cy = i.ravel()
                        cv2.circle(frame_bkp, (cx, cy), 2, 255, -1)

                if display:
                    cv2.imdisplay('frame', frame_bkp)

                prev_frame_gray = frame_gray


            if display:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            rate.sleep()
