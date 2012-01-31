#include <ros/ros.h>
#include <joy/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h>
#include "serialswitch.hpp"
#include <ros/console.h>
#include <cstdio>


using namespace std;

//SerialSwitch *sp;

class ElektronTeleopJoy {
public:
	ElektronTeleopJoy();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);


	ros::NodeHandle nh_;

	int linear_, angular_;
	double l_scale_, a_scale_;
	ros::Publisher vel_pub_;
//	ros::Publisher state_pub_;
	ros::Subscriber joy_sub_;

	int stateCount;
	

};

ElektronTeleopJoy::ElektronTeleopJoy() {

	stateCount = 0;

	nh_.param("axis_linear", linear_, 1);
	nh_.param("axis_angular", angular_, 0);
	nh_.param("scale_angular", a_scale_, 1.0);
	nh_.param("scale_linear", l_scale_, 0.23);

	vel_pub_ = nh_.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
	
//	state_pub_ = nh_.advertise<std_msgs::Int16> ("state1",1);

	joy_sub_ = nh_.subscribe<sensor_msgs::Joy> ("joy", 10, &ElektronTeleopJoy::joyCallback, this);


  //  std::string dev = "/dev/ttyUSB1";
  //  sp = new SerialSwitch(dev);

}

void ElektronTeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	ROS_INFO("joyCallback");
//	fprintf(stdout,"joyCallback \n");

	geometry_msgs::Twist vel;
	vel.angular.z = a_scale_ * joy->axes[angular_];
	vel.linear.x = l_scale_ * joy->axes[linear_];
	vel_pub_.publish(vel);
	
	/*
	std_msgs::Int16 state;

	if(joy->buttons[4]==1){
		++stateCount;
		state.data = stateCount;

		if (sp->isConnected()) {
			if(ros::ok()){
				sp->update();
			}
		} else {
			ROS_ERROR("Connection to device  failed");
		}
	}
    	state_pub_.publish(state);
    	*/
	
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "piotrek_teleop_node");
	ElektronTeleopJoy elektron_teleop;

	ros::spin();
}
