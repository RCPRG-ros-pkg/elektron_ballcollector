#include <ros/ros.h>
#include <joy/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include "serialswitch.hpp"
#include <ros/console.h>
#include <cstdio>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

//SerialSwitch *sp;

class ElektronTeleopJoy {
public:
	MoveBaseClient ac;
	ElektronTeleopJoy();

	void onHoover();
	void offHoover();
	void switchHoover();
	void goForward(float dist);


private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);


	ros::NodeHandle nh_;

	int linear_, angular_;
	double l_scale_, a_scale_;
	ros::Publisher vel_pub_;
	ros::Publisher state_pub_;
	ros::Subscriber joy_sub_;

	ros::Publisher go_forward_robot_pub_;

	int stateCount;
	

};

ElektronTeleopJoy::ElektronTeleopJoy():ac("move_base", true) {

	stateCount = 0;

/*	nh_.param("axis_linear", linear_, 1);
	nh_.param("axis_angular", angular_, 0);
	nh_.param("scale_angular", a_scale_, 1.0);
	nh_.param("scale_linear", l_scale_, 0.23);*/

	a_scale_ = 0.2;
	l_scale_ = 0.2;
	nh_.param("axis_linear", linear_, 1);
	nh_.param("axis_angular", angular_, 0);
	nh_.param("scale_angular", a_scale_, 0.2);
	nh_.param("scale_linear", l_scale_, 0.2);


	vel_pub_ = nh_.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
	state_pub_ = nh_.advertise<std_msgs::Int16> ("hoover_state",1);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy> ("joy", 10, &ElektronTeleopJoy::joyCallback, this);

	go_forward_robot_pub_ = nh_.advertise< std_msgs::Float32>("/robot_go_straight",1);



  //  std::string dev = "/dev/ttyUSB1";
  //  sp = new SerialSwitch(dev);

}

void ElektronTeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
//	ROS_INFO("joyCallback");
//	fprintf(stdout,"joyCallback \n");

/*
	if(joy->axes[0] != 0 || joy->axes[1] != 0 || joy->axes[2] != 0 || joy->axes[3] != 0){

	}*/
	

	geometry_msgs::Twist vel;
	vel.angular.z = a_scale_ * joy->axes[angular_];
	vel.linear.x = l_scale_ * joy->axes[linear_];
	vel_pub_.publish(vel);


	if(joy->buttons[4]==1){
//		ROS_INFO("Button fire");

		 // move_base_msgs::MoveBaseGoal goal;
		//  ROS_INFO("Sending goal");
		  ac.cancelAllGoals ();

	}
	
	if(joy->buttons[5]==1){
		onHoover();
	}

	if(joy->buttons[7]==1){
		offHoover();
	}

/*	if(joy->buttons[6]==1){
		goForward(0.3);
	}*/


	
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



void ElektronTeleopJoy::onHoover() {
	std_msgs::Int16 state;
	state.data = 1;
	state_pub_.publish(state);

}

void ElektronTeleopJoy::offHoover() {
	std_msgs::Int16 state;
	state.data = 0;
	state_pub_.publish(state);

}

void ElektronTeleopJoy::switchHoover() {

	std_msgs::Int16 state;
	state.data = 2;
	state_pub_.publish(state);

}

void ElektronTeleopJoy::goForward(float dist){
	std_msgs::Float32 go;
	go.data = dist;
	go_forward_robot_pub_.publish(go);
}

