# open-robot
The main ROS package of open source robot


# Open-robot ros stack (indigo)
Put the ros stack(open_robot) in /src directory of you ros workspace.
In order to compile the code successfuly,you need install some drivers and ROS packages.

## dependencies
Follow this page to install ROS in ubuntu 14.04, Desktop-Full Install is recommended.
	http://wiki.ros.org/indigo/Installation/Ubuntu

Install ROS basic packages. Run the command
	"sudo apt-get install ros-indigo-joy ros-indigo-depthimage-to-laserscan ros-indigo-gmapping ros-indigo-map-server ros-indigo-amcl ros-indigo-move-base ros-indigo-dwa-local-planner ros-indigo-smart-battery-msgs".

Install OpenGL Utility Toolkit.Run the command "sudo apt-get install freeglut3-dev".

## RealSense

This open robot depends on RealSense camera R200, it needs an additional library, refer to the guide at
	 https://github.com/IntelRealSense/librealsense
to complete the driver installation.


## submodule

Several ROS packages are maintained in git sub module, use the following command to fetch the source code

		cd src/open_robot
		git submodule init
		git submodule update
After success, the submodule realsense, rplidar_ros etc is stored in the same directory of open robot



optional ros packages:
Install kinect driver.Run the command "sudo apt-get install ros-indigo-freenect*"
Install xtion driver.Run the command "sudo apt-get install ros-indigo-opennni2*"
Install ros hectoring mapping package.Run the command "sudo apt-get install ros-indigo-hectormapping"


# License

Copyright 2016 Intel Corporation

Licensed under the BSD
