#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <std_msgs/Float32.h>

using namespace ros;


class SubscribeAndPublish
{
	public:
	SubscribeAndPublish()
	{
		//Topic you want to publish
		pub_ = n_.advertise<std_msgs::Float32>("control_num", 100);

		//Topic you want to subscribe
		sub_ = n_.subscribe("distance", 100, &SubscribeAndPublish::pid_func, this);
	}

	void pid_func(const std_msgs::Float32::ConstPtr& msg)
	{
		static float prev_error = 0;
		static float accum_error = 0;
		int Kp, Ki, Kd, setpoint;
		float error, d_error, P, I, D, PID;
		ROS_INFO("I heard: [%f]", msg->data);
		n_.param("/gains/P", Kp, 0);
		n_.param("/gains/I", Ki, 0);
		n_.param("/gains/D", Kd, 0);
		n_.param("set_point", setpoint, 20);
	
		error = msg->data - setpoint;
		P = Kp * error;
	
		accum_error += error;
		I = Ki * accum_error;
	
		d_error = error - prev_error; 
		D = Kd * d_error;
	
		PID = P + I + D;
		std_msgs::Float32 msg_out;

		msg_out.data = PID;
		ROS_INFO("I say: [%f]", PID);
		pub_.publish(msg_out);
	}

	private:
		NodeHandle n_; 
		Publisher pub_;
		Subscriber sub_;
};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
	init(argc, argv, "pid_block");

	SubscribeAndPublish PID_block;

	spin();
	
	return 0;
}
