## CMPUT 412 Winter 2018 Competition 3: Race with GMapping and AMCL, By Jae-yeon (Leo) Yoon and Sumeyye Aydin ##

The competition report can be found ![here]()!

To download the necessary files:
```
git clone https://github.com/leoyoon17/CMPUT-412-Competition-3

```
Note: Don't forget to connect the usb cables from the Kobuki base, the
Asus Xtion Pro Live, and the logitech controller to your computer! (or ssh)

Note: This program does not have a launch file as Leo's computer was unable to create a workspace package with "catkin_make", thus launchfiles would not run properly. 

Note: This program was test on a specific location and results may vary due to the specificity of the map.

To run the program, first open 6 terminals (or tmux windows, etc):
===========================================================================

On the first terminal:
* $ roscore

On the second terminal:
* $ roslaunch turtlebot_bringup minimal.launch

On the third terminal:
* $ roslaunch turtlebot_teleop logitech.launch

On the fourth terminal:
* $ roslaunch turtlebot_navigation amcl_demo.launch map_file:= [complete directory to map.yaml file]

* Example:
* $ roslaunch turtlebot_navigation amcl_demo.launch map_file:=file:///home/jaeyeon/Documents/School/CMPUT412/Competition%203/map.yaml

On the fifth terminal:
* $ roslaunch turtlebot_rviz_launchers view_navigation.launch

On the sixth terminal:
* change directory to /Competition 3/
* chmod +x nav.py
* $ ./nav.py

* After launching ./nav.py , the robot should move to the first way point, when 'ready' press the 'B' button to begin.