## CMPUT 412 Competition 1: Pursuit & Evasion, By Jae-yeon (Leo) Yoon and Sumeyye Aydin ##

Objectives
==========

The general objective of the competition was to utilize the optical sensor on the Turtlebot, as well as smooth motion. More specifically, our team was to improve the wander behavior of our robot to swiftly manouver around the environment for the evasion portion of the competition. For the pursuit section of the competition, we were to use the optical sensor (Asus Xtion Pro Live) to track a moving target.


Background
==========
For the pursuit section of the competition we utilized the packages provided by OpenCV (Open source Computer Vision) to track the target.
Initially, the image collected by the optical sensor is converted/bridged into the array of values.
![Image Conversion](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython?action=AttachFile&do=get&target=cvbridge3.png)  

```python 
bridge = CvBridge() 
```

To actually track a moving target, the concept of Optical Flow was utilized (the pattern of apparnet motion of image objects between two consecutive frames caused by the movement of object or camera. Its good to note that the cv2 library comes with with code to easily incorporate optical flow to track targets.

![Optical Flow](https://docs.opencv.org/3.3.1/optical_flow_basic1.jpg) 

```python
cv2.calcOpticalFlowPyrLK(prev_frame_gray, frame_gray, corners, None, **lk_params)
```

Some paramaters had to be initialized to make the Lucas Kanade Optical Flow function properly.
(Note: the general concept behind the Lucas Kanade Optical Flow revolves around taking a 3x3 patch around the point. So all the 9 points have the same motion.)

```python
# To track Optical Flow (the pattern of apparent motion of images between two consecutive frames)
# Parameters for ShiTomasi corner detection
# Concept & code from https://docs.opencv.org/3.3.1/d7/d8b/tutorial_py_lucas_kanade.html
feature_params = dict( maxCorners = 100,
        qualityLevel = 0.3,
        minDistance = 7,
        blockSize = 7 )

# Parameters for Lucas Kanade Optical Flow
lk_params = dict( winSize  = (15,15),
        maxLevel = 2,
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
```

Finally, the concept of velocity ramping was used to ensure that we follow the target with little jerky movements.

Hypothesis
==========

During the pursuit portion of the competition, we wanted to see if it was possible to reliably follow our opposing robot by utilizing the optical sensor. Our hypothesis for this portion was that following another robot was possible.

Conversely, during the evasion portion of the competition, we wanted to find out if we could successfully navigate around the given environment by utilizing the optical sensor. Our hypothesis for this portion was that evading the given environment was possible.

Materials
=========
The optical sensor which was used for tracking/vision was the Asus Xtion Pro Live.
The base of the robot was the Kobuki Turtlebot, which includes a differential drive and bumpers on its perimeter.
A Logitech F710 controller was used to switch between on and sleep modes as needed for the competition.
We made basic modifications to the stock robot in which we were provided:
  * The top frame was removed to make room for the laptop which would run the robot. This was done mostly to lower the center of gravity.
  * The position of the optical sensor was lowered to capture more of the floor, as aiming the camera higher would result in unwanted tracking.
  * A seat-belt-esque strap was placed on the laptop to ensure that it would not fall off during movement
  
The following nodes were used in the program to run the robot:
* follower (for all of the pursuit section)
* movement (for the velocity smoothing in the pursuit section)
* wander (for the evasion section) 
  
 Method
 ======
 
 The initial concept of using OpenCV's library to track the robot was from a previous competition's program. Their program was a good place to start, but changed had to be made to bring it to competition specifications. (we could not get it to run via the given launch file, so we had to source some sections of the code to fit our needs) (https://github.com/BrianEverRoboticsTeam/Evasion-Pursuit-Competition)
 
 Movement was based off the textbook's (Programming Robots with ROS) velocity ramping code.

 Based on the testing of our pursuit robot, we realized that if the opposing robot gets out of range, then it cannot follow other bot. Thus, we decided to work on the speed, since we believed that it was the key to victory.
As a result, we did not use a velocity smoother to get faster even though it was resulting in jerky movements. 
Besides the speed we used LaserScan to calculate the distance between the object in front of the robot and itself. Also, instead of only turning and moving, we implemented the robot to move and turn at the sametime if the distance is not less than the threshold as well as not nan. 
 
 Results
 =======
 
 Fortunately, the robot performed well during the competition (albeit, thanks to malfunctions on the other teams' programs). The evasion bot was able to out manouver most of the comptetition due to its lack of velocity smoothing and continuous obstacle checking. For the pursuit section, our robot was able to track the opposing robot, and follow it well. Although, one of the pursuit rounds, our robot failed to reverse out of the incoming robot's path.

We can see the results of the evasion section here:
![Evasion](https://github.com/leoyoon17/CMPUT-412-Competition-1/blob/master/docs/evasion.mp4)

And the results of the pursuit section is here:
![Pursuit](https://github.com/leoyoon17/CMPUT-412-Competition-1/blob/master/docs/pursuit.mp4)
 
Discussion
==========
Easily, the portion of the competition where we experienced the most success was during the evasion section. This was most likely due to our robot's ability to detect walls while it was moving, and make incremental changes to its trajectory rather than coming to a full stop to make a turn.

Unfortunately, during its second run on pursuit, it was not able to track the evading robot. This was most likely due to the evading robot approaching from a blind spot on the optical sensor, thus we had lost track of them (most likely too close for the optical sensors to detect the other robot).

During testing, we had noticed that while the robot was tracking a moving target, any reflective surfaces would catch its attention and begin tracking that instead. We do not quite know the cause of this phenomenon yet, but we were lucky enough that no reflective surfaces were in the optical sensor's sights during the actual competition.

In the end, the most important aspect for success during this competition was how FAST our robot would move (accelerating, decelerating, turning, etc). During testing, our environment was much larger than the actual competition grounds, so we had calibrated our velocity to be rather high. But thanks to the velocity smoothing, this was no issue.

On a final note, our robot was disfunctional when we had faced the TA's robot. This was likely due to the laptop shutting down during waiting for our turn.

Moving forward, better organization of code would have been helpful (separating large tasks into individual tasks), although it was not a priority as the basic functions had to be complete before the competition.

Conclusions
============
In the end, our goal was to learn about our Turtlebots and utilizing the sensors at hand to complete the given task. From the results of the competition, we can come to terms with our performance, but there definitely is room for improvement. Our robot successfully evaded and pursued the opposing robots at least once, which means that the algorithm is functional to some extent.

Some improvements would include dealing with reflective surfaces, and improved targetting (as we failed to follow a target which got to our blindspot).

The observed results of the competition supports our hypothesis, illustrating that we are able to use the optical sensor to track the environment, and track a moving target.
