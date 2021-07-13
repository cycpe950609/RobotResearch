# Setup Turtlebot3

// TODO

# Setup remote PC

// TODO

# Setup Cartographer
* Method is same on [Turtlebot3 emanual](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node)

	* sudo apt-get install ninja-build libceres-dev libprotobuf-dev protobuf-compiler libprotoc-dev
	* cd ~/catkin_ws/src
	* git clone https://github.com/googlecartographer/cartographer.git
	* git clone https://github.com/googlecartographer/cartographer_ros.git
	* cd ~/catkin_ws
	* src/cartographer/scripts/install_proto3.sh
	* rm -rf protobuf/
	* rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:xenial
	* catkin_make_isolated --install --use-ninja
	* source ~/catkin_ws/install_isolated/setup.bash
