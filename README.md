# Open-robot



## Purpose
Open-robot is the main ROS package of [Open Source Robot](https://github.com/open-robot).  
[Open Source Robot](https://github.com/open-robot) now provides 2D-SLAM and Navigaiton functionalities. You can also find the [hardware](https://github.com/open-robot/Hardware) and [Arduino firmware](https://github.com/open-robot/ArduinoFirmware) repository in that project. If you want to build your own robot, our repositories may save your time.  
More information about Open Source Robot is on our [web page.](http://www.ros-robot.com/)

## Demo Video
[![Open source mobile robot video](https://i.ytimg.com/vi/PIggmwuGv_8/hqdefault.jpg)](https://www.youtube.com/embed/PIggmwuGv_8)

## Dependencies
1. [Install ROS](http://wiki.ros.org/indigo/Installation/Ubuntu) in ubuntu 14.04, Desktop-Full Install is recommended.  	
2. Some ROS packages are needed. Run the following command to install.

		sudo apt-get install ros-indigo-joy ros-indigo-smart-battery-msgs 
		sudo apt-get install ros-indigo-gmapping ros-indigo-robot-pose-ekf ros-indigo-map-server
		sudo apt-get install ros-indigo-costmap-2d ros-indigo-dwa-local-planner ros-indigo-move-base ros-indigo-amcl
		sudo apt-get install ros-indigo-librealsense

## Building
1. Clone open-robot in /src directory of your ros workspace.

		git clone https://github.com/open-robot/open-robot.git

2. Several ROS packages are maintained in git sub module, use the following command to fetch the source code.

		cd src/open_robot
		git submodule init
		git submodule update
	After success, the submodule realsense, rplidar_ros etc is stored in the same directory of open robot.

3. Compile open-robot In your catkin workspace

		catkin_make


## Start
1. Make sure the usb device has permission.   If you are, like us, using cp2102 as your USB to UART bridge, write a 51-usb-tty.rules file with the following content.

		SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="users", MODE="0666"

 Then run the following command in terminal to copy 51-usb-tty.rules to /etc/udev/rules.d and reboot.

		sudo cp -p 51-usb-tty.rules /etc/udev/rules.d/51-usb-tty.rules
		sudo reboot
 
2. To configure ROS Environment for open-robot, run the following command in terminal.

		echo "source <Your catkin workspace path>/devel/setup.bash" >> ~/.bashrc
	
3. Start open-robot.

		roslaunch robot_bringup demo_start.launch


## 2D-SLAM on open-robot

1. To run [Google Cartographer](https://github.com/googlecartographer/cartographer_ros) on open-robot, refer to this [README_open_robot.rst](https://github.com/open-robot/cartographer_ros/blob/open-robot/README_open_robot.rst) in [our repository](https://github.com/open-robot/cartographer_ros).  

2. Aside from Cartographger, you can use gmapping. After launching [demo_start.launch](https://github.com/open-robot/open-robot/blob/master/robot_bringup/launch/demo_start.launch), run the following command.

		roslaunch robot_slam gmapping.launch

3. (Optional) Start rviz in a remote computer. About how to [run ROS across multiple machines.](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)


## 2D-Navigation on open-robot
1. Start open-robot.

		roslaunch robot_bringup demo_start.launch

2. Launch nav.launch after a map having been built.
 
		roslaunch robot_nav nav.launch

3. (Optional) Start rviz in a remote computer. About how to [run ROS across multiple machines.](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)



## RealSense(optional)

Visual-SLAM is to be enabled soon. So now, Open-robot just uses RealSense camera R200 to capture images. It needs an additional library, refer to the guide at
	 https://github.com/IntelRealSense/librealsense
to complete the driver installation.


# License

Copyright 2016 Intel Corporation

Licensed under the BSD
