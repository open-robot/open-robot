# Open-robot



## Purpose
Open-robot is the main ROS package of Open Source Robot. 
You can also find the hardware and Arduino firmware repositories in that project. 
Suggestion: If you want to build your own robot, our repos may save your time.


## Dependencies
1. Follow this page to install ROS in ubuntu 14.04, Desktop-Full Install is recommended.
	
	http://wiki.ros.org/indigo/Installation/Ubuntu

2. Some ROS packages are needed. Run the following command to install.

	"sudo apt-get install ros-indigo-joy ros-indigo-depthimage-to-laserscan ros-indigo-gmapping ros-indigo-map-server ros-indigo-amcl ros-indigo-move-base ros-indigo-dwa-local-planner ros-indigo-smart-battery-msgs ros-indigo-robot-pose-ekf ros-indigo-map-server ros-indigo-costmap-2d".


## Building
1. Clone open-robot in /src directory of your ros workspace

		git clone https://github.com/open-robot/open-robot.git


2. Several ROS packages are maintained in git sub module, use the following command to fetch the source code

		cd src/open_robot
		git submodule init
		git submodule update
	After success, the submodule realsense, rplidar_ros etc is stored in the same directory of open robot

3. Compile open-robot In your catkin workspace

		catkin_make


## How to Start
1. Make sure the usb device has permission

     If you are, like us, using cp2102 as your USB to UART bridge, write a 51-usb-tty.rules file with the following content "SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="users", MODE="0666"". Then copy this file to /etc/udev/rules.d and reboot.

2. To configure ROS Environment for open-robot, run the following command

		echo "source "Your catkin workspace path"/devel/setup.bash" >> ~/.bashrc
	
3. Start open-robot

		roslaunch robot_bring start.launch


## 2D-SLAM on open-robot

1. To run Google Cartograpger on open-robot, refer to the README.rst in our repo(forked from Cartographer).
	https://github.com/open-robot/cartographer_ros


2. Aside from Cartograpger, you can use gmapping. After launching the start.launch, run the following command.

		roslaunch robot_nav gmapping.launch


## RealSense(optional)

Visual-SLAM is to be enabled soon. So now, Open-robot just uses RealSense camera R200 to capture images. It needs an additional library, refer to the guide at
	 https://github.com/IntelRealSense/librealsense
to complete the driver installation.


# License

Copyright 2016 Intel Corporation

Licensed under the BSD
