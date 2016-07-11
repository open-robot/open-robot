#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_P_U 0x35
#define KEYCODE_P_D 0x36

class TeleopTurtle
{
	public:
		TeleopTurtle();
		void keyLoop();
	private:
		ros::NodeHandle nh_;
		double linear_, angular_, l_scale_, a_scale_;
		ros::Publisher twist_pub_;
};

TeleopTurtle::TeleopTurtle():
	linear_(0),
	angular_(0),
	l_scale_(2.0),
	a_scale_(2.0)
{
	nh_.param("teleop_keyboard/translate", linear_, linear_);
	nh_.param("teleop_keyboard/rotate", angular_, angular_);

	twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_turtle");
	TeleopTurtle teleop_turtle;

	signal(SIGINT,quit);

	teleop_turtle.keyLoop();

	return 0;
}


void TeleopTurtle::keyLoop()
{
	char c;
	bool dirty=false;

	// get the console in raw mode                                                              
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file                         
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move the robot.");

	for(;;) {
		// get the next event from the keyboard  
		if(read(kfd, &c, 1) < 0) {
			perror("read():");
			exit(-1);
		}

		ROS_DEBUG("value: 0x%02X\n", c);

		geometry_msgs::Twist twist;

		switch(c) {
		case KEYCODE_L:
			ROS_DEBUG("LEFT");
			twist.linear.x = 0;
			twist.linear.y = linear_;
			twist.angular.z = 0;
			dirty = true;
			break;
		case KEYCODE_R:
			ROS_DEBUG("RIGHT");
			twist.linear.x = 0;
			twist.linear.y = -linear_;
			twist.angular.z = 0;
			dirty = true;
			break;
		case KEYCODE_U:
			ROS_DEBUG("UP");
			twist.linear.x = linear_;
			twist.linear.y = 0;
			twist.angular.z = 0;
			dirty = true;
			break;
		case KEYCODE_D:
			ROS_DEBUG("DOWN");
			twist.linear.x = -linear_;
			twist.linear.y = 0;
			twist.angular.z = 0;
			dirty = true;
			break;
		case KEYCODE_P_U:
			ROS_DEBUG("TURN LEFT");
			twist.linear.x = 0;
			twist.linear.y = 0;
			twist.angular.z = angular_;
			dirty = true;
			break;
		case KEYCODE_P_D:
			ROS_DEBUG("TURN LEFT");
			twist.linear.x = 0;
			twist.linear.y = 0;
			twist.angular.z = -angular_;
			dirty = true;
			break;
		}

		if(dirty ==true) {
			twist_pub_.publish(twist);    
			dirty=false;
		}
	}
}
