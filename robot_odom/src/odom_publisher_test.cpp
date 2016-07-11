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
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "robot_battery/battery.h"
//#include "sensor_msgs/BatteryState.h"

//#define TTYDEV "/dev/ttyUSB0"
#define TTYDEV "/dev/ttyS4"
#define BUADRATE B115200

//m/s trans to circle/min, formula: 1m/s = 1*60/(2*pi*0.029) circle/min. wheel radius 0.029m
#define VEL_TRANS 329.4531
//circle/min trans to m/s,formula: 1circle/min = 1*(2*pi*0.029)/60 m/s.
#define INVERSE_VEL_TRANS 0.00304
//#define L 0.11
#define PI 3.1415926

float linear_x,linear_y,angular_z;     //linear_x and linear_y is m/s , angular_z is rad/s
char read_buf[512];
ros::Time current_time;
ros::Time last_time;
double odometry_x,odometry_y,odometry_th;
int fdtty;
double L;
//
float add_th = 0.0;
ros::Time current_time1;
ros::Time last_time1;

nav_msgs::Odometry n_odom;

geometry_msgs::TransformStamped n_odom_trans;
float voltage;


int initialserialport(const char* strtty)
{
	struct termios newtio,oldtio;
	int fdtty;
	if(strtty==0)
	{
		fdtty = open(TTYDEV,O_RDWR | O_NOCTTY | O_NDELAY);
	}
	else
	{
		fdtty = open(strtty,O_RDWR | O_NOCTTY | O_NDELAY);
	}		
	//1.open tty device.	
	if(-1==fdtty)
	{
		printf("no device found\n");
		return -1;
	}	
	//2.set port.
	if(tcgetattr(fdtty,&oldtio))
	{
		printf("[%s,%d]fail to get tty attr.\n",
				__FILE__,__LINE__);		
	}

	if(fcntl(fdtty,F_SETFL,0))
	{
		printf("fail to block the fdev.\n");
	}
	if(isatty(STDIN_FILENO)==0)
	{
		printf("standard input is not a terminal device.\n");
	}

	//3. init the new tio
	memset(&newtio,0,sizeof(struct termios));
	//set char size
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	//set data bits
	newtio.c_cflag |= CS8;
	newtio.c_cflag &= ~PARENB;
	//set io speed
	cfsetospeed(&newtio,BUADRATE);
	cfsetispeed(&newtio,BUADRATE);
	//set stop bit
	//newtio.c_cflag &= ~CSTOPB;
	newtio.c_cflag |= CSTOPB;

	//set min char to receive.	
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fdtty,TCIFLUSH);
	if((tcsetattr(fdtty,TCSANOW,&newtio))!=0)
	{
		printf("com set error");
		return -1;
	}
	printf("Initializing serial done.\n");	
	return fdtty;
}

void encoder_data_package(float* v1,float* v2,float* v3,unsigned char* frame)
{
	unsigned char a = 0;
	frame[0] = 0x55;
	frame[1] = 0xAA;
	frame[2] = 0x01;
	memcpy(&frame[3],v1,sizeof(float));
	memcpy(&frame[7],v2,sizeof(float));
	memcpy(&frame[11],v3,sizeof(float));
	for(int i=3;i<15;i++)
	{
		a += frame[i];
	} 
	frame[15] = a;
	frame[16] = 0x0D;
}


void PublishOdometry(int *diff_encoder, float *v_motor, tf::TransformBroadcaster *broadcaster,
		geometry_msgs::TransformStamped *odom_trans, ros::Publisher *odom_pub)
{
	current_time = ros::Time::now(); 
	float wheel_dis[3];
	float x,y,th;
	float vx, vy, vth;
	wheel_dis[0] = diff_encoder[0]*PI*58*0.001/330/4;
	wheel_dis[1] = diff_encoder[1]*PI*58*0.001/330/4;
	wheel_dis[2] = diff_encoder[2]*PI*58*0.001/330/4;
        //printf("%f %f %f\n",v_motor[0],v_motor[1],v_motor[2]);
	//printf("pub_odom %f, %f, %f \n", wheel_dis[0], wheel_dis[1], wheel_dis[2]);
	x = (wheel_dis[0]-wheel_dis[1])/1.732;
	y = (2*wheel_dis[2]-wheel_dis[0]-wheel_dis[1])/3.0;
	th = -(wheel_dis[0]+wheel_dis[1]+wheel_dis[2])/(3*L);
	//printf("%f %f %f\n",v_motor[0],v_motor[1],v_motor[2]);
	for(int i=0;i<3;i++)
		{v_motor[i] = v_motor[i]*INVERSE_VEL_TRANS;}
	//double dt = (current_time - last_time).toSec();
	vx = (v_motor[0]-v_motor[1])/1.732;
	vy = (2*v_motor[2]-v_motor[0]-v_motor[1])/3.0;
	vth = -(v_motor[0]+v_motor[1]+v_motor[2])/(3*L);

	odometry_th += th;
	odometry_x += (x * cos(odometry_th) - y * sin(odometry_th));
	odometry_y += (y * cos(odometry_th) + x * sin(odometry_th));

	if(odometry_th > 2*PI)
		odometry_th -= 2*PI;
	if(odometry_th < -2*PI)
		odometry_th += 2*PI;

	geometry_msgs::Quaternion odom_quat;
	odom_quat = tf::createQuaternionMsgFromYaw(odometry_th);

//	double test_th;
//	test_th = tf::getYaw(odom_quat);
//	printf("                                  yaw_th:%f\n",test_th*180/3.1415926);

	//filling the odometry
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
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
 
	//velocity
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = vth;		

	// publishing the odometry
	odom_pub->publish(odom);
	//printf("%f %f %f\n",v_motor[0],v_motor[1],v_motor[2]);
	//printf("%f %f %f\n",vx,vy,vth);
	last_time = current_time;
}

void send(int fdtty,float linear_x, float linear_y, float angular_z) 
{
	int i;
	int n;
	unsigned char cmd[17] ;
	static int copy_fdtty=fdtty;
	float wheel_left,wheel_right,wheel_back;
	wheel_left =( linear_x*0.8660254 -linear_y*0.5 -angular_z*L)*VEL_TRANS;
	wheel_back =( linear_y                         -angular_z*L)*VEL_TRANS;
	wheel_right=(-linear_x*0.8660254 -linear_y*0.5 -angular_z*L)*VEL_TRANS;	
	encoder_data_package(&wheel_left,&wheel_right,&wheel_back,cmd);
	n = write(copy_fdtty, cmd, 17);
	for(i=0;i<10;i++);

}


void* serialport_receive(void* pstfd)
{
	int fdtty = *((int*)pstfd);
	long rx_len=0;
	unsigned char rx_flag[4]={0x55,0xAA,0x00,(unsigned char)'\0'};
	int i,j;
	char *p;
	unsigned char temp[24],temp1[4],temp2[4],temp3[4],temp4[4],temp5[4],temp6[4];
	float motor[3]={0,0,0},*tempmotor1,*tempmotor2,*tempmotor3,*tempmotor4,*tempmotor5,*tempmotor6;
	float tempmotor1_old = 0,tempmotor2_old = 0,tempmotor3_old = 0;
	
	ros::NodeHandle nh;
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
	tf::TransformBroadcaster broadcaster;
	geometry_msgs::TransformStamped odom;
	//sensor_msgs::BatteryState BatteryState;

	double vx = 0.0;
	double vy = 0.0;
	double vth = 0.0;

	int new_encoder[3];
	int old_encoder[3]={0,0,0};
	int diff_encoder[3];	
	float v_motor[3] = {0,0,0};

	//To renew the old_encoder
    	while(1) {		
		memset(read_buf,0,29*sizeof(char));
		rx_len=read(fdtty,read_buf,29*sizeof(char));
		//for(int i=0;i<100000;i++)
		//{int aa = 0;}	
		if(rx_len<=0)
			continue;

		const char * rx_str =(char*)rx_flag;
		p=strstr(read_buf,rx_str);

		if(p)  //only if p is not null
		{
			unsigned char a = 0;
			//for(int i=0;i<29;i++)
				//printf("%x ",(unsigned char)p[i]);
			//printf("\n");
			for(int i=3;i<27;i++) {
				a += (unsigned char)p[i];
			//	printf("%x ",(unsigned char)p[i]);
			}
			//printf("a = %x\n",a);
			
			//85,170 are the hexadecimal type of 0x55,0xAA
			if((unsigned char)p[0] != 85 || (unsigned char)p[1] != 170) {
				printf("package start error!!!\n");
				continue;
			}
			if((unsigned char)p[2] == 3 ) {
				printf("the battery message comes\n");
				continue;	
			}	
			if(a != (unsigned char)p[27]) {
				
				printf("a = %x, p[27] = %x\n",a,p[27]);				
				printf("checksum error!!!\n");
				continue;
			}
			
			if((unsigned char)p[28] != 13)  //13 is the hexadecimal type of 0x0D
			{
				printf("package end error!!!\n");
				continue;	
			}			

			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					temp[i*4+j]=(unsigned char)p[3+j+(4*i)];
				}
			}

			for(i=0;i<4;i++)
			{
				temp1[i] = temp[  i];	
				temp2[i] = temp[4+i];
				temp3[i] = temp[8+i];		
			}
			tempmotor1 = (float*)temp1;
			tempmotor2 = (float*)temp2;
			tempmotor3 = (float*)temp3;
			//printf("%f %f %f\n",*tempmotor1,*tempmotor2,*tempmotor3);

                        
                        old_encoder[0] = *tempmotor1;
			old_encoder[1] = *tempmotor2;
			old_encoder[2] = *tempmotor3;
		        printf("initialized the old_encoder!\n");
			break;
				
		}
	
	}

	while(1) {			
		memset(read_buf,0,29*sizeof(char));
		rx_len=read(fdtty,read_buf,29*sizeof(char));
		for(int i=0;i<50000;i++)
		{int aa = 0;}	
		if(rx_len<=0)
			continue;

		const char * rx_str =(char*)rx_flag;
		p=strstr(read_buf,rx_str);

		if(p) {
			unsigned char a = 0;
			//for(int i=0;i<29;i++)
				//printf("%x ",(unsigned char)p[i]);
			//printf("\n");
			for(int i=3;i<27;i++) {
				a += (unsigned char)p[i];
			//	printf("%x ",(unsigned char)p[i]);
			}
			//printf("a = %x\n",a);
			
			if((unsigned char)p[2] == 3) {
				printf("the battery message comes!\n");
				if((unsigned char)p[0] != 85 || (unsigned char)p[1] != 170) {
					printf("package start error!!!\n");
					continue;
				}
			
				if(a != (unsigned char)p[27]) {
					/*
		                        for(int i=0;i<17;i++) {
		                        printf("%x ",(unsigned char)p[i]);
					}
					printf("\n");
					*/
					printf("a = %x, p[27] = %x\n",a,p[27]);				
					printf("checksum error!!!\n");
					continue;
				}
			
				if((unsigned char)p[28] != 13)  //13 is the hexadecimal type of 0x0D
				{
					printf("package end error!!!\n");
					continue;	
				}			

				for(i=0;i<6;i++)
				{
					for(j=0;j<4;j++)
					{
						temp[i*4+j]=(unsigned char)p[3+j+(4*i)];
					}
				}

				for(i=0;i<4;i++)
				{
					temp1[i] = temp[  i];	
					temp2[i] = temp[4+i];
					temp3[i] = temp[8+i];
					temp4[i] = temp[12+i];
					temp5[i] = temp[16+i];
					temp6[i] = temp[20+i];		
				}
				/* BatteryState.voltage = *((float*)temp1);
				BatteryState.current = *((float*)temp2);
				BatteryState.charge = *((float*)temp3);
				BatteryState.capacity = *((float*)temp4);
				BatteryState.design_capacity = *((float*)temp5);
				BatteryState.percentage = *((float*)temp6);
				*/
				voltage = *((float*)temp1);
				printf("voltage:%f\n",voltage);
					
			}
			
			else if((unsigned char)p[2] == 0) {
				//85,170 are the hexadecimal type of 0x55,0xAA
				if((unsigned char)p[0] != 85 || (unsigned char)p[1] != 170) {
					printf("package start error!!!\n");
					continue;
				}
			
				if(a != (unsigned char)p[27]) {
					/*
		                        for(int i=0;i<17;i++) {
		                        printf("%x ",(unsigned char)p[i]);
					}
					printf("\n");
					*/
					printf("a = %x, p[27] = %x\n",a,p[27]);				
					printf("checksum error!!!\n");
					continue;
				}
			
				if((unsigned char)p[28] != 13)  //13 is the hexadecimal type of 0x0D
				{
					printf("package end error!!!\n");
					continue;	
				}			

				for(i=0;i<6;i++)
				{
					for(j=0;j<4;j++)
					{
						temp[i*4+j]=(unsigned char)p[3+j+(4*i)];
					}
				}

				for(i=0;i<4;i++)
				{
					temp1[i] = temp[  i];	
					temp2[i] = temp[4+i];
					temp3[i] = temp[8+i];
					temp4[i] = temp[12+i];
					temp5[i] = temp[16+i];
					temp6[i] = temp[20+i];		
				}
				tempmotor1 = (float*)temp1;
				tempmotor2 = (float*)temp2;
				tempmotor3 = (float*)temp3;
				tempmotor4 = (float*)temp4;
				tempmotor5 = (float*)temp5;
				tempmotor6 = (float*)temp6;
				//printf("%f %f %f\n",*tempmotor4,*tempmotor5,*tempmotor6);

		                
		                new_encoder[0] = *tempmotor1;
				new_encoder[1] = *tempmotor2;
				new_encoder[2] = *tempmotor3;

				diff_encoder[0] = new_encoder[0] - old_encoder[0];
				diff_encoder[1] = new_encoder[1] - old_encoder[1];
				diff_encoder[2] = new_encoder[2] - old_encoder[2];
			

		                //the wheel going 1 round causes 1320 impulses
				//13200 means the wheel goes about 10 rounds whitch is unreasonable

				old_encoder[0] = new_encoder[0];
				old_encoder[1] = new_encoder[1];
				old_encoder[2] = new_encoder[2];
				v_motor[0] = *tempmotor4;
				v_motor[1] = *tempmotor5;
				v_motor[2] = *tempmotor6;
			
				PublishOdometry(diff_encoder,v_motor, &broadcaster, &odom, &odom_pub);	
			}	
		}
		for(int i=0;i<50;i++)
			usleep(1000);
	}
}

void chatterCallback(const boost::shared_ptr<geometry_msgs::Twist const>& msg)
{
	send(fdtty, msg->linear.x, msg->linear.y, msg->angular.z);
}

void chatterCallback1(const boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const>& msg)
{
	send(0,0,0,0);	

	ros::NodeHandle nh;
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
	tf::TransformBroadcaster odom_broadcaster;

	odometry_x = msg->pose.pose.position.x;
	odometry_y = msg->pose.pose.position.y;
	odometry_th = tf::getYaw(msg->pose.pose.orientation);
	current_time = ros::Time::now(); 

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odometry_th);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_footprint";
	
	//set the position
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

	//set the velocity
	odom.twist.twist.linear.x = 0.0;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = 0.0;		

	// publishing the message
	odom_pub.publish(odom);

	last_time = current_time;
	//printf("The initial pose set: x=%lf; y=%lf; th=%lf\n",odometry_x,odometry_y,odometry_th);
}


void chatterCallback2(const boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const>& msg)
{
        tf::TransformBroadcaster odom_broadcaster;
	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();

	float new_x, new_y;
	new_x = msg->pose.pose.position.x;
	new_y = msg->pose.pose.position.y;

        geometry_msgs::Quaternion odom_quat;       
	odom_quat = msg->pose.pose.orientation;
/*
        n_odom_trans.header.stamp = current_time;
        n_odom_trans.header.frame_id = "odom";
        n_odom_trans.child_frame_id = "base_footprint";
        n_odom_trans.transform.translation.x = new_x;
        n_odom_trans.transform.translation.y = new_y;
        n_odom_trans.transform.translation.z = 0.0;
        n_odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(n_odom_trans);

*/
        n_odom.header.stamp = current_time;
	n_odom.header.frame_id = "odom";
	n_odom.child_frame_id = "base_footprint";
        n_odom.pose.pose.position.x = new_x;
        n_odom.pose.pose.position.y = new_y;
        n_odom.pose.pose.position.z = 0.0;
        n_odom.pose.pose.orientation = odom_quat;
	
	n_odom.twist.twist.linear.x = 0.0;
	n_odom.twist.twist.linear.y = 0.0;
	n_odom.twist.twist.linear.z = 0.0;
	n_odom.twist.twist.angular.x = 0.0;
	n_odom.twist.twist.angular.y = 0.0;
	n_odom.twist.twist.angular.z = 0.0;
	//new_odom_pub.publish(n_odom);
}

bool feedback(robot_battery::battery::Request  &req,
         robot_battery::battery::Response &res)
{
  res.sum = voltage;
  
  ROS_INFO("sending back response: [%lf]", ( float)res.sum);
  printf("sending back response:x=%lf\n", ( float)res.sum);
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_publisher");
	ros::NodeHandle n;
	pthread_t thread1, thread2;  
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("battery", feedback);

	int threadwhether;
	fdtty = initialserialport(0);
	threadwhether = pthread_create(&thread1, NULL, &serialport_receive, (void*)(&fdtty));
	last_time = ros::Time::now();
	last_time1 = ros::Time::now();
	ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, chatterCallback);
	ros::Subscriber sub1 = n.subscribe("/initialpose", 1000, chatterCallback1);
        ros::Subscriber sub2 = n.subscribe("/robot_pose_ekf/odom_combined", 1000, chatterCallback2);
	
	n.param<double>("odom_pub/robot_radius", L, 0.105);

	ros::Rate loop_rate(10);

	ros::Publisher new_odom_pub = n.advertise<nav_msgs::Odometry>("new_odom", 100);

	//set the default tf between odom and base_link,publish the default odometry message
	odometry_x = 0.0;
	odometry_y = 0.0;
	odometry_th = 0.0;
	current_time = ros::Time::now(); 
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odometry_th);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_footprint";

	//set the position
	odom.pose.pose.position.x = odometry_x;
	odom.pose.pose.position.y = odometry_y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.twist.twist.linear.x = 0.0;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.linear.z = 0.0;
	odom.twist.twist.angular.x = 0.0;
	odom.twist.twist.angular.y = 0.0;
	odom.twist.twist.angular.z = 0.0;		

	// publishing the message
	odom_pub.publish(odom);
	last_time = current_time;
	//set end

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
		new_odom_pub.publish(n_odom);
	}

	close(fdtty);
	return 0;
}

