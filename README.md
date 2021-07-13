# RobotResearch
大學專題 - Mobile robot control using Machine Learning

# Environment
* Ubuntu 18.04
* ROS1 Melodic

# src/ 
All ROS package

# Packages : 
***只有以下幾個packages有用，其餘皆是開發過程的測試，不保證正確性***
* robot_navigation : 整個專題最終控制機器人的程式
* ros_logger : 利用ROS的Bag file紀錄程式執行時的變數值
* set_goal_gui_rviz : 用 RVIZ 控制 Turtlebot3 目標 / 顯示SLAM地圖 / 顯示模擬光達的Point Cloud

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
	5. roslaunch turtlebot3_bringup turtlebot3_robot.launch (在Turtlebot3 上)
	6. roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer configuration_basename:=turtlebot3_lds_2d_gazebo.lua open_rviz:=false
		1. slam_methods:=cartographer : 使用Cartographer執行SLAM
		2. configuration_basename:=turtlebot3_lds_2d_gazebo.lua : 如果使用gazebo模擬，則須使用專屬的設定（見turtlebot3/turtlebot3_slam/config/turtlebot3_lds_2d_gazebo.lua），否則會有「The IMU frame must be colocated with the tracking frame. Transforming linear acceleration into the tracking frame will otherwise be imprecise.」錯誤
		3. open_rviz:=false : 我們有set_goal_gui_rviz顯示地圖，故cartographer不須打開rviz
* 另外也可以直接用本目錄下的start_simulation.sh(要控制Gazebo的機器人時)以及start_turtlebot3.sh(要控制Turtlebot3時)
	* NOTE: 因為roscore啟動時間較慢，故需要先啟動roscore一遍

# TODO:
