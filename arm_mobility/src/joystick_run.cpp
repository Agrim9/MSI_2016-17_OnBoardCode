#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>

#define MAX_ACTUATOR_SPEED 150

ros::Publisher mob_pub;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	std::vector<float> inp1 = joy -> axes;
	std::vector<int> inp2 = joy -> buttons;
	
	std::vector<double> out(3, 0);
	out[0] = MAX_ACTUATOR_SPEED * inp1[7];
	out[1] = inp2[3];
	out[2] = inp2[0]; 

	std_msgs::Float64MultiArray outMsg;
	outMsg.data = out;
	mob_pub.publish(outMsg);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "joystick_driver");
	ros::NodeHandle _nh;
	
	mob_pub = _nh.advertise<std_msgs::Float64MultiArray>("/arm/arm_directives", 10);
	ros::Subscriber joy_sub = _nh.subscribe("/joy", 10, joyCallback);
	ros::Rate loop_rate(10);

	while(1) {

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
