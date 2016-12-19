#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/String.h"
ros::Publisher ard_pub;

void navCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {

	std::vector<double> inp = msg -> data;
	float ActuatorSpeed = inp[0];
	float MotorForward = inp[1];
	float MotorBackward = inp[2];

	std::vector<double> out(3, 0);
	out[0] = ActuatorSpeed;
	out[1] = MotorForward;
	out[2] = MotorBackward;

	std_msgs::Float64MultiArray outMsg;
	outMsg.data = out;
	ard_pub.publish(outMsg);
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "arm_driver");
	ros::NodeHandle _nh;
	ros::Publisher chatter_pub = _nh.advertise<std_msgs::String>("chatter", 1000);
	ard_pub = _nh.advertise<std_msgs::Float64MultiArray>("/arm/ard_directives", 100);
	ros::Subscriber nav_sub = _nh.subscribe("/arm/arm_directives", 100, navCallback);
	ros::Rate loop_rate(10);

	ros::spin();

	return 0;
}
