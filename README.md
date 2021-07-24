# path-planner-LQR-controller

Status : Active

# Prerequisites
ROS Melodic, Ubuntu 18.04, Turtlebot3 packages (https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation and 
https://github.com/ROBOTIS-GIT/turtlebot3_msgs)

# Brief Intro

This project aims to develop a LQR controller to control Turtlebot 3 and navigate it in a pure pursuit path.

# Execution
Please copy the package to your workspace_folder/src and follow the steps listed.

•Execute catkin_make in your work space folder

•Execute source devel/setup.bash

•Execute roscore

•Execute roslaunch path-planner lqrTurtlebot.launch for default spawn location (origin)

•Please pass in the arguments for x_pos, y_pos, z_pos by following this line 
roslaunch path-planner lqrTurtlebot.launch x_pos:=value y_pos:= value z_pos:= value

In order to change the robot, execute export TURTLEBOT3_MODEL=model where model includes burger,waffle and wafflePi
