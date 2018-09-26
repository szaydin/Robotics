## CMPUT 412 Winter 2018 Competition 4: Egg Hunting, By Jae-yeon (Leo) Yoon and Sumeyye Aydin ##

The competition report can be found ![here](https://github.com/leoyoon17/CMPUT-412-Competition-4/blob/master/docs/Competition%204%20Report.pdf)!

To download the necessary files:
```
git clone https://github.com/leoyoon17/CMPUT-412-Competition-4

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
* $ roslaunch map_server map_server map.yaml

On the fourth terminal:
* $ roslaunch turtlebot_navigation amcl_demo.launch map_file:= [complete directory to map.yaml file]

* Example:
* $ roslaunch turtlebot_navigation amcl_demo.launch map_file:=file:///home/jaeyeon/Documents/School/CMPUT412/Competition%204/map.yaml

On the fifth terminal:
* $ roslaunch turtlebot_rviz_launchers view_navigation.launch

On the sixth terminal:
* $ roslaunch sound_play soundplay_node.launch

On the seventh terminal:
* $ roslaunch usb_cam usb_cam-test.launch

On the eighth terminal:
* Note: the comp4.launch file must be placed into ar_track_alvar's launch folder.
* $ roslaunch ar_track_alvar comp4.launch

On the ninth terminal:
* change directory to /Competition 4/src
* chmod +x stateMachine.py
* $ ./stateMachine.py

* After launching ./stateMachine.py , the robot should spin to localize itself, then move to the waypoints.
