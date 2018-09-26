## CMPUT 412 Winter 2018 Competition 2: Run, Robot Run, By Jae-yeon (Leo) Yoon and Sumeyye Aydin ##

The competition report can be found ![here](https://github.com/leoyoon17/CMPUT-412-Competition-2/blob/master/docs/report.md)!

To download the necessary files:
```
git clone https://github.com/leoyoon17/CMPUT-412-Competition-2

```
Note: Don't forget to connect the usb cables from the Kobuki base, the
Asus Xtion Pro Live, and the logitech controller to your computer! (or ssh)

Note: This program does not have a launch file as Leo's computer was unable to create a workspace package with "catkin_make", thus launchfiles would not run properly.

Note: The camera must be placed relatively high on the robot and pointed at a downward angle for the program to run properly (which can be seen in the competition video listed on the competition report).

To run the program, first open 5 terminals (or tmux windows, etc):
===========================================================================

On the first terminal:
* $ roscore

On the second terminal:
* $ roslaunch turtlebot_bringup minimal.launch

On the third terminal:
* $ roslaunch turtlebot_bringup 3dsensor.launch

On the fourth terminal:
* $ roslaunch turtlebot_teleop logitech.launch

On the fifth terminal:
* change directory to /Competition 2/src
* chmod +x follower_p.py 
* ./follower_p.py cmd_vel:=cmd_vel_mux/input/teleop

* Finally, press the 'X' button on the logitech controller to start or stop the program!






