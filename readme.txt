Directory Structure:
    reactive-bot -> ROS package for our robot
        include -> files that need to be included when compiling our package (automatic created by ROS)
        launch -> launch file for gazebo simulation, opening our world and spawning the robot
        models -> models for the walls
        src -> source code. Contains code for controlling the robot.
        worlds -> gazebo world file which include the walls.

Requirements:
    Have ROS Noetic installed.
    Have Gazebo installed (is intalled with ROS Noetic).
    Have the following packages installed:
        - turtlebot3_msgs: https://github.com/ROBOTIS-GIT/turtlebot3_msgs
        - turtlebot3: https://github.com/ROBOTIS-GIT/turtlebot3
        - turtlebot3_simulations: https://github.com/ROBOTIS-GIT/turtlebot3_simulations

How to Compile:
    Source both the ROS and the workspace (the path can be different) setup scripts:
        - source /opt/ros/melodic/setup.bash
        - source ~/catkin_ws/devel/setup.bash

    Compile with catkin:
        - catkin_make

How to execute:
    Add the following enviromental variables:
        - export TURTLEBOT3_MODEL=burger
        - export GAZEBO_MODEL_PATH=~/catkin_ws/src/ROBO-2020/reactive-robot/models:{GAZEBO_MODEL_PATH}
    
    Launch Gazebo simulation:
        - roslaunch reactive-robot reactive-robot.launch

    On another terminal, run our controller:
        - rosrun reactive-robot robot_controller