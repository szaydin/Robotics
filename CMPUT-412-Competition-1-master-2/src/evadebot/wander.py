#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Joy
import numpy as np
import random
import math


sleep_bot = True
g_twist = Twist() # default to 0


def sleep_switch(msg):
    global sleep_bot

    if msg.buttons[2]==1:
      print ("X Button was pressed")
      sleep_bot = (sleep_bot==False)

    print(sleep_bot)

def got_twist(msg):
    global g_twist
    g_twist = msg

def scan_callback(msg): #Scan_callback gets the distance parameters from the camera and keep it in the list, find the minimum distance to the object which let robot to turn or move
  global get_min_range, g_min_ind
  get_min_range = min(msg.ranges) #getting the minimum distance 

  depths_ahead = []
  depths_all = []

  dist_idx = 0
  for dist in msg.ranges: #Getting rid of the nan outputs for each of the range messages and put them all in depths
    if not np.isnan(dist):
      depths_all.append(dist)

      angle = (abs(dist_idx - 320) / 320.0 ) * 29 #Calculating the angle to make a movement
      y_dist = dist * math.sin(math.radians(angle)) # calculating the y distance and append them to the list 
      if y_dist < 0.28:
        depths_ahead.append(dist)
    dist_idx += 1

  if (len(depths_ahead)!=0): #if the y distance list is not empty, then we need to grab the minimum distance to 
    get_min_range = min(depths_ahead)

  if (len(depths_all)!=0):
    g_range_all_min = min(depths_all)

  try:
    g_min_ind = msg.ranges.index(g_range_all_min)
  except:
    g_min_ind = -1

  print("range:",get_min_range,"index:",g_min_ind)

g_min_ind = 320 # initial to midle point
g_last_twist = Twist() # initial to 0
get_min_range = 1 # anything to start
g_stop_distance = 1.2

driving_forward = True
start_turning = False
only_turning = False

joy_sub = rospy.Subscriber('joy', Joy, sleep_switch)
cmd_vel_with_dead_bot_switch = rospy.Subscriber('cmd_vel_safe', Twist, got_twist)
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')
 
rate = rospy.Rate(10)



while not rospy.is_shutdown():

  if (sleep_bot != True): #Activating/deactivating the bot if the key is pressed on the controller
    if driving_forward:

      if (get_min_range < g_stop_distance or np.isnan(get_min_range)): # while moving if the range ahead is smaller than the threshold or if the camera cannot get any info, which is caused by being too close, then it does not move forward
        driving_forward = False


    else: 

      if get_min_range > g_stop_distance: # Otherwise it moves (getting minimum range ahead higher than the threshold) forward 
        driving_forward = True 


    if start_turning: # robot starts turning with this command based on the min_range
      if (get_min_range < 0.7 or np.isnan(get_min_range)): # 
        only_turning = True
    else:
      if get_min_range > 0.7:
        only_turning = False
        start_turning = False

    if driving_forward: # if driving forward, then robot moves forward with 0.7 velocity
      g_last_twist.linear.x = 0.7
      g_last_twist.angular.z = 0
      start_turning = False
      only_turning = False
    elif not start_turning: #Robot moves forward with lower speed and turn at the sametime depends on the laser scan message. If there is a obstacle at the right then turns left or vice versa.
      g_last_twist.linear.x = 0.45
      start_turning = True

      if g_min_ind > 320:
        g_last_twist.angular.z = -2
      elif g_min_ind>=0:
        g_last_twist.angular.z = 2


    elif only_turning: # Robot does not move forward if the distance is smaller than 0.7 and there are obstacles at the both side of the robot
      g_last_twist.linear.x = 0
      if g_last_twist.angular.z == 0:
          g_last_twist.angular.z = random.uniform(-3.14, 3.14)

    cmd_vel_pub.publish(g_last_twist)

    rate.sleep()
