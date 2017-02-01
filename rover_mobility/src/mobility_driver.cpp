#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/String.h"
ros::Publisher drive_pub;

void driveCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {

	std::vector<double> inp = msg -> data;
	float linSpeed = inp[0];
	float angSpeed = inp[1]; 

	std::vector<double> out(10, 0);
	for(int i=0; i<6;i++) {
		out[i] = linSpeed;
	}
	out[6] = angSpeed;
	out[7] = -angSpeed;
	out[8] = -angSpeed;
	out[9] = angSpeed;

	std_msgs::Float64MultiArray outMsg;
	outMsg.data = out;
	ard_pub.publish(outMsg);
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "mobility_driver");
	ros::NodeHandle _nh;
	drive_pub = _nh.advertise<std_msgs::Float64MultiArray>("/rover/drive_directives", 100);
	ros::Subscriber drive_sub = _nh.subscribe("/rover/control_directives", 100, driveCallback);
	ros::Rate loop_rate(10);

	ros::spin();

	return 0;
}
