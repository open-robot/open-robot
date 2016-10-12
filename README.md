# open-robot
The main ROS package of open source robot


# Open-robot ros stack (indigo)
Put the ros stack(open_robot) in /src directory of you ros workspace.
In order to compile the code successfuly,you need install some drivers and ROS packages.

## Dependencies
1. Follow this page to install ROS in ubuntu 14.04, Desktop-Full Install is recommended.
	http://wiki.ros.org/indigo/Installation/Ubuntu

2. Install ROS basic packages. Run the following command
	"sudo apt-get install ros-indigo-joy ros-indigo-depthimage-to-laserscan ros-indigo-gmapping ros-indigo-map-server ros-indigo-amcl ros-indigo-move-base ros-indigo-dwa-local-planner ros-indigo-smart-battery-msgs ros-indigo-robot-pose-ekf".

3. Install OpenGL Utility Toolkit.Run the command "sudo apt-get install freeglut3-dev".

## RealSense

This open robot depends on RealSense camera R200, it needs an additional library, refer to the guide at
	 https://github.com/IntelRealSense/librealsense
to complete the driver installation.


## Submodule

Several ROS packages are maintained in git sub module, use the following command to fetch the source code

		cd src/open_robot
		git submodule init
		git submodule update
After success, the submodule realsense, rplidar_ros etc is stored in the same directory of open robot



optional ros packages:
Install kinect driver.Run the command "sudo apt-get install ros-indigo-freenect*"
Install xtion driver.Run the command "sudo apt-get install ros-indigo-opennni2*"
Install ros hectoring mapping package.Run the command "sudo apt-get install ros-indigo-hectormapping"


## How to start

1. Make sure the usb device has permission.

     If you are, like us, using cp2102 as your USB to UART bridge, write a 51-usb-tty.rules file with the following content "SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="users", MODE="0666"". Then copy this file to /etc/udev/rules.d and reboot.

2. Configuring ROS Environment for open-robot.

     Run the command "echo "source "Your catkin workspace path"/devel/setup.bash" >> ~/.bashrc"

3. Start open-robot.

     Run the command "roslaunch robot_bring start.launch"

# License

Copyright 2016 Intel Corporation

Licensed under the BSD
