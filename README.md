# path-planner-LQR-controller

Status : Active

# Prerequisites
ROS Melodic, Ubuntu 18.04, Turtlebot3 packages


# Brief Intro

This project aims to develop a LQR controller to control Turtlebot 3 and navigate it in a pure pursuit path.

# Execution
Please copy the package to your workspace_folder/src and follow the steps listed.

•Execute `catkin_make` in your work space folder

•Execute `source devel/setup.bash`

•Execute `roscore`

•Execute `roslaunch path-planner lqrTurtlebot.launch` for default spawn location (origin)

•Please pass in the arguments for x_pos, y_pos, z_pos by following this line 
`roslaunch path-planner lqrTurtlebot.launch x_pos:=value y_pos:= value z_pos:= value`

In order to change the robot, execute `export TURTLEBOT3_MODEL=model` where model includes burger,waffle and wafflePi

# References

[ROS Melodic Installation](http://wiki.ros.org/melodic/Installation)  
[ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)  
[Turtlebot3 Quick start](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)  
[Turtlebot3 Simulation packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)  
[Turtlebot3 messages](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)  
J.Snider.,Automatic Steering Methods for Autonomous Automobile Path Tracking, 2011.  
R Siegwart, I R Nourbaksh,Introduction to Autonomous Mobile Robots, MIT Press, 2004.  
G. Oriolo, A. D. Luca, and M. Vendittelli, WMR control via dynamic feedback linearization: design, implementation, and experimental validation, IEEE Transactions on Control Systems Technology, vol. 10, no. 6, pp. 835–852, 2002.  
