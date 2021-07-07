#!/bin/sh
export ROS_HOSTNAME=192.168.43.147 && export ROS_MASTER_URI=http://192.168.43.147:11311 
# export ROS_HOME=$PWD
# Start Turtlebot3simulation
tmux new-session -d -s turtlebot3simulation -n 'roscore' 'roscore'
echo 'Start roscore'
sleep 4 # wait for roscore start
# tmux new-window -t turtlebot3simulation:1 -n 'Turtlebot3 Remote' 'roslaunch turtlebot3_bringup turtlebot3_remote.launch > test.txt'
echo 'Start Turtlebot3 Remote'
tmux new-window -t turtlebot3simulation:2 -n 'SetGoalServer' 'rosrun set_goal_gui_rviz set_goal_gui_server'
echo 'Start SetGoalServer'
tmux new-window -t turtlebot3simulation:3 -n 'SetGoalGUI' 'rosrun rviz rviz -d $(rospack find set_goal_gui_rviz)/rviz/set_goal_gui_rviz.rviz'
echo 'Start SetGoalGUI'
# tmux new-window -t turtlebot3simulation:4 -n 'RobotNavigation' 'roslaunch robot_navigation robot_navigation.launch'
tmux new-window -t turtlebot3simulation:4 -n 'RobotNavigation' 'roslaunch robot_navigation robot_navigation.launch > robot_navigation.log'
# tmux new-window -t turtlebot3simulation:4 -n 'RobotNavigation' 'roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch'

echo 'Start RobotNavigation'
# tmux new-window -t turtlebot3simulation:5 -n 'Cartographer' 'roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer open_rviz:=false'
tmux new-window -t turtlebot3simulation:5 -n 'GMapping' 'roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping open_rviz:=false configuration_basename:=turtlebot3_lds_2d_gazebo.lua'
echo 'Start GMapping'
tmux new-window -t turtlebot3simulation:6 -n 'Turtlebot3' 'ssh user@turtlebot3.local'
echo 'Start Turtlebot3 SSH'



tmux select-window -t turtlebot3simulation:6
tmux -2 attach-session -t turtlebot3simulation
