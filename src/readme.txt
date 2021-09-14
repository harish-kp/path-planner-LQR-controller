Prerequisites - ROS Melodic, Ubuntu 18.04, Turtlebot3 packages (https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)

Please copy the package to your workspace_folder/src and follow the steps listed.

•Execute catkin_make in your work space folder

•Execute source devel/setup.bash

•Execute roscore

•Execute roslaunch path-planner lqrTurtlebot.launch for default spawn location (origin)

•Please pass in the arguments for x_pos, y_pos, z_pos by following this line 
roslaunch path-planner lqrTurtlebot.launch x_pos:=value y_pos:= value z_pos:= value

In order to change the robot, execute export TURTLEBOT3_MODEL=model where model includes burger,waffle and wafflePi

#Reference

Eigen Tensor matrix access (https://stackoverflow.com/questions/48650751/eigentensor-how-to-access-matrix-from-tensor)
Eigen::Tensor to Eigen::Matrix conversion (https://stackoverflow.com/questions/48795789/eigen-unsupported-tensor-to-eigen-matrix)
