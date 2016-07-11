#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
class TeleopJoy
{
	public:
		TeleopJoy();

	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

		ros::NodeHandle nh_;

		double linear_, angular_;
	//	double l_scale_, a_scale_;
		ros::Publisher vel_pub_;
		ros::Subscriber joy_sub_;

};
TeleopJoy::TeleopJoy():
	linear_(1),
	angular_(2)
{
	nh_.param("teleop_joystick/translate", linear_, linear_);
	nh_.param("teleop_joystick/rotate", angular_, angular_);
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist vel;
//	vel.angular.z = a_scale_*joy->axes[angular_];
//	vel.linear.x = l_scale_*joy->axes[linear_];
	if(joy->buttons[6] == 1)
		vel.angular.z = angular_;
	if(joy->buttons[7] == 1)
		vel.angular.z = -angular_;	
	vel.linear.x = linear_*joy->axes[9];
	vel.linear.y = linear_*joy->axes[8];
	vel_pub_.publish(vel);
	vel.angular.z = 0;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_teleop_joy");
	TeleopJoy teleop_joy;

	ros::spin();
}
