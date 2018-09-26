## CMPUT 412 Competition 2: Run, Robot Run, By Jae-yeon (Leo) Yoon and Sumeyye Aydin ##

Objectives
==========

The general objective of the competition was to utilize the optical sensor (Asus Xtion Pro Live) and the Turtlebot's movement to quickly navigate around a course via lane detection.


Background
==========
For the program, we utilized the packages provided by OpenCV (Open source Computer Vision) to track lanes and turns/corners on the course. Initially, the image collected by teh optical sensor is converted/bridged into the array of values.

![Image Conversion](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython?action=AttachFile&do=get&target=cvbridge3.png) 

```python 
bridge = CvBridge() 
```

Then, the raw image is converted into HSV to further isolate the white lanes.

```python
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
lower_white = numpy.array([ 0,  0,  255-self.threshold])
upper_white = numpy.array([255, self.threshold, 255])
```

Hypothesis
==========

During the competition, we wanted to see if it was possible to reliably follow the course/lanes, and navigate the course at a relatively high speed via lane detection. 

Materials
=========
The optical sensor which was used for tracking/vision was the Asus Xtion Pro Live.
The base of the robot was the Kobuki Turtlebot, which includes a differential drive and bumpers on its perimeter.
A Logitech F710 cotroller was ued to switch between on and sleep mode as needed for the competition.
We made basic modifications to the stock robot in which we were provided:
  * The top frame was removed to make room for the laptop which would run the robot. This was done mostly to lower the center of gravity.
  * The position of the optical sensor was raised drastically to capture both lanes at a reasonable distance away
  * A seat-belt-esque strap was placed on the laptop to ensure that it would not fall off during movement

The only node that was used in the program to run the robot was
* follower

Method
======

 The initial concept of utilizing OpenCV's library to track one line was taken from the text Programming Robots with Ros. To track both the lanes, two masks were used to detect the left and right lanes separately. The threshold in which the robot sees a lane as 'white' can be adjusted via manipulating the ``` self.threshold ``` value.

```python
search_top = 3*h/5 - 30
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
```

 The robot's movement was based on its position between the lanes. After calculating the midpoint between the two lanes, the robot can adjust itself accordingly based on the ``` self.err ``` value.

 The angular.z velocity and linear.x velocity both depend on ``` self.dampening ``` which which is based off an adujstable ``` self.compensation ``` value, as well as the turning angle of the corner.
 As the corner becomes sharper, the speed is lowered to compensate, as well as increasing the turning rate.

 The turning angle was based on pythagorian principles.
![Pythagoras](https://github.com/leoyoon17/CMPUT-412-Competition-2/blob/master/docs/pythagoras.gif)


Results
========

 Fortunately, the robot performed well during the competition. Our first place victory was thanks to the good racing line while tackling corners. The robot would stick to the inside line of the corner to shave as much time as possible (this was a consequence of turning in via turning angle).

We can see the results of the final race ![here](https://github.com/leoyoon17/CMPUT-412-Competition-2/blob/master/docs/race.mp4)

Discussion
==========
Easily, the most difficult problem to address with this competition was how to deal with sharp corners, as the camera would lose track of one of the lanes coming to close to the inside lane. This problem was alleviated by introducing realizing that the loss of a lane was inevitable during a sharp turn, thus we must compensate more heavily during said turn until we find two lanes again.

Another problem whilst tackling this competition was how to navigate throughout the course efficiently so that we can shave as much time as possible - as all the robots had the same top speed. The solution to the problem was to take concepts from motorsports into perspective, and utilize it in our final product. by sticking to the inside lane during relatively sharp turns (the apex of the turn), we can reduce unnecesary distance traveled.

![Corners](https://github.com/leoyoon17/CMPUT-412-Competition-2/blob/master/docs/corner.jpg)

Conclusions
===========
Ultimately, our goal was to furthe understance OpenCV and applying it to robotics with the given sensors. From the results for the competition, we can come to terms with our performance, but as always, there is room for improvement. Our robot sucessfully ran the given course in a timely manner, thus the algorithm was functional to some extent.

Future improvements would include further mimicing a racecar's racing line to decrease lap times, and a more consistent line detection.

The observed results of the competition supports our hypothesis, illustrating that we are able to use the optical sensor to detect lanes, and move along the given track in a timely manner.


