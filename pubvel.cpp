#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

using namespace ros;

int main(int argc, char **argv)
{
	init(argc, argv, "publish_velocity");
	NodeHandle nh;

	Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
	
	srand(time(0));
	
	Rate rate(2);
	
	while(ok())
	{
		geometry_msgs::Twist Pubmsg;
		Pubmsg.linear.x = double(rand())/double(RAND_MAX);
		//Pubmsg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;
		Pubmsg.angular.z = -(double(rand())/double(RAND_MAX));
		
		pub.publish(Pubmsg);
		
		ROS_INFO_STREAM( "Sending random velocity command:"
 			<< " linear=" << Pubmsg.linear.x
			<< " angular=" << Pubmsg.angular.z);
		rate.sleep();
	}
}