#include "ros/ros.h"
#include "robot_battery/battery.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "battery_client");


  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<robot_battery::battery>("battery");
  robot_battery::battery srv;

  if (client.call(srv))
  {
    ROS_INFO("Sum: %lf", ( float)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service battery");
    return 1;
  }

  return 0;
}
