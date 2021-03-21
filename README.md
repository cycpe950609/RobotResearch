# RobotResearch
大學專題 - Mobile robot control using Machine Learning

# Environment
* Ubuntu 18.04
* ROS1 Melodic

# src/ 
All ROS package

# Packages
* square_walk_control : 控制機器人走正方形，每五秒轉向
* random_controllor : 控制機器人隨機走，每五秒轉向
* motion_stable_control : 計算機器人的速度與角速度（Asymptotic Stability）並前往目標（Go2Goal）
* avoid_obstacle_control : 在安全距離內，Turtlebot3會倒行(倒退) 
* set_goal_gui : 用 Tkinter 控制 Turtlebot3 目標
* set_goal_gui_rviz : 用 RVIZ 控制 Turtlebot3 目標

# Setup
See [setup.md](setup.md)

# Note
* 啟動順序
	1. roscore 
	2. rosrun motion_stable_control Go2GoalServer 
	3. roslaunch turtlebot3_bringup turtlebot3_remote.launch **(很重要！！！不然會讀不到機器人)**
	4. roslaunch set_goal_gui_rviz set_goal_gui.launch (似乎RVIZ v1.13.16 的OGRE v1.9.0在nVidia上有機會處發bug而segmentation fault，故使用以下兩條指令分別開啟server和rviz)
		1. rosrun rviz rviz -d $(rospack find set_goal_gui_rviz)/rviz/set_goal_gui_rviz.rviz
		2. rosrun set_goal_gui_rviz set_goal_gui_server
	5. roslaunch turtlebot3_bringup turtlebot3_robot.launch (在Turtlebot3 上)mark

