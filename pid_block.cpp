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
		pub_ = n_.advertise<geometry_msgs::Twist>("control_spd", 100);

		//Topic you want to subscribe
		sub_ = n_.subscribe("distance", 100, &SubscribeAndPublish::pid_func, this);
	}

	void pid_func(const std_msgs::Float32::ConstPtr& msg)
	{
		static double prev_error = 0;
		static double accum_error = 0;
		double Kp, Ki, Kd, setpoint;
		double error, d_error, P, I, D, PID, output;
		ROS_INFO("I heard: [%f]", msg->data);
		n_.param("/gains/P", Kp, 0.0);
		n_.param("/gains/I", Ki, 0.0);
		n_.param("/gains/D", Kd, 0.0);
		n_.param("set_point", setpoint, 15.0);
	
		error = (double)msg->data - (double)setpoint;
		P = Kp * error;
	
		accum_error += error;
		I = Ki * accum_error;
	
		d_error = error - prev_error;
		prev_error = error;
		D = Kd * d_error;
	
		PID = P + I + D;
		geometry_msgs::Twist msg_out;

		output = saturate (PID);
		msg_out.linear.x = output;
		ROS_INFO("I say: [%f]", output);
		pub_.publish(msg_out);
	}

	double saturate (double value)
	{
		float clip_val;
		if (value > max)
		{
			clip_val = max;
		}
		else if (value < (-1.0 * max))
		{
			clip_val = (-1.0 * max);
		}
		else
		{
			clip_val = value;
		}

		return clip_val;
	}

	private:
		double max = 100.0;
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
