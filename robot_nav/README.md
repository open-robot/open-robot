#robot_nav

Make sure you had run the roscore and bringup.

##mapping
Run the command "roslaunch robot_nav slam_gmapping.launch",than move the robot.When finished,run the command "rosrun map_server map_saver -f your_map_name" to save the map.

##navigation
Run the command "roslaunch robot_nav nav.launch",set the "2D Pose Estimate" and the "2D Nav Goal" in rviz.