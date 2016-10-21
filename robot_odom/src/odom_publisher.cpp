#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <malloc.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <math.h>
#include <linux/input.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


//m/s trans to circle/min, formula: 1m/s = 1*60/(2*pi*0.029) circle/min. wheel radius 0.029m
#define VEL_TRANS 329.4531
//circle/min trans to m/s,formula: 1circle/min = 1*(2*pi*0.029)/60 m/s.
#define INVERSE_VEL_TRANS 0.00304
//#define L 0.11
#define PI 3.1415926

ros::Time current_time;
//ros::Time last_time;
double odometry_x=0;
double odometry_y=0;
double odometry_th=0;
double L=0;
nav_msgs::Odometry odom;

void PublishOdometry(int *diff_encoder)
{
	float wheel_dis[3];
	float x,y,th;
	wheel_dis[0] = diff_encoder[0]*PI*58*0.001/330;
	wheel_dis[1] = diff_encoder[1]*PI*58*0.001/330;
	wheel_dis[2] = diff_encoder[2]*PI*58*0.001/330;
	x = (wheel_dis[0]-wheel_dis[1])/1.732;
	y = (2*wheel_dis[2]-wheel_dis[0]-wheel_dis[1])/3.0;
	th = -(wheel_dis[0]+wheel_dis[1]+wheel_dis[2])/(3*L);

	odometry_th += th;
	odometry_x += (x * cos(odometry_th) - y * sin(odometry_th));
	odometry_y += (y * cos(odometry_th) + x * sin(odometry_th));

	if(odometry_th > 2*PI)
		odometry_th -= 2*PI;
	if(odometry_th < -2*PI)
		odometry_th += 2*PI;

	geometry_msgs::Quaternion odom_quat;
	//printf("theta : %f\n",odometry_th);
	odom_quat = tf::createQuaternionMsgFromYaw(odometry_th);

	//odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_footprint";
	// position
	odom.pose.pose.position.x = odometry_x;
	odom.pose.pose.position.y = odometry_y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	odom.pose.covariance[0] = 0.1;
	odom.pose.covariance[7] = 0.1;
	odom.pose.covariance[14] = 999999;
	odom.pose.covariance[21] = 999999;
	odom.pose.covariance[28] = 999999;
	odom.pose.covariance[35] = 0.05;
}

void wheel_speed_cb(const boost::shared_ptr<std_msgs::Float32MultiArray const>& msg)
{
	current_time = ros::Time::now();
	ros::NodeHandle nh;
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;
    double v_motor[3];
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_footprint";
    for(int i=0;i<3;i++)
		{v_motor[i] = msg->data[i]*INVERSE_VEL_TRANS;}
	//double dt = (current_time - last_time).toSec();
	vx = (v_motor[0]-v_motor[1])/1.732;
	vy = (2*v_motor[2]-v_motor[0]-v_motor[1])/3.0;
	vth = -(v_motor[0]+v_motor[1]+v_motor[2])/(3*L);

	//velocity
	odom.twist.twist.linear.x = vx; 
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = vth;

	odom.pose.covariance[0] = 0.1;
	odom.pose.covariance[7] = 0.1;
	odom.pose.covariance[14] = 999999;
	odom.pose.covariance[21] = 999999;
	odom.pose.covariance[28] = 999999;
	odom.pose.covariance[35] = 0.05;

	// publishing the odometry
	odom_pub.publish(odom);
}

int old_encoder[3]={0,0,0};
bool old_encoder_init = false;

void encoder_cnts_cb(const boost::shared_ptr<std_msgs::Float32MultiArray const>& msg){

	int new_encoder[3];
	if (!old_encoder_init) {
		old_encoder[0] = msg->data[0];
		old_encoder[1] = msg->data[1];
		old_encoder[2] = msg->data[2];
		old_encoder_init = true;
	}

	int diff_encoder[3];
	
	new_encoder[0] = msg->data[0];
	new_encoder[1] = msg->data[1];
	new_encoder[2] = msg->data[2];

	diff_encoder[0] = new_encoder[0] - old_encoder[0];
	diff_encoder[1] = new_encoder[1] - old_encoder[1];
	diff_encoder[2] = new_encoder[2] - old_encoder[2];

	old_encoder[0] = new_encoder[0];
	old_encoder[1] = new_encoder[1];
	old_encoder[2] = new_encoder[2];

	PublishOdometry(diff_encoder);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_publisher");
	ros::NodeHandle n;
	n.param<double>("odom_pub/robot_radius", L, 0.105);
	ros::Publisher odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 100);
	ros::Subscriber sub = n.subscribe("/encoder_cnts", 1000, encoder_cnts_cb);
	ros::Subscriber sub_1 = n.subscribe("/speed_wheel", 1000, wheel_speed_cb);
/*
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
*/
	ros::spin();
	return 0;
}
