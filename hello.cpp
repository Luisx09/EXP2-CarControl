#include <ros/ros.h>

using namespace ros;

int main(int argc, char **argv)
{
	init(argc, argv, "hello_ros");

	NodeHandle nh;

	ROS_INFO_STREAM("Hello, ROS!");
}
