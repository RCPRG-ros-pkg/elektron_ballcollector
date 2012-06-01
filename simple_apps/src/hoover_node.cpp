#include <ros/ros.h>
#include <std_msgs/Int16.h>

#include "serialswitch.hpp"


SerialSwitch *sp;

class HooverNode {

	ros::NodeHandle nh_;
	ros::Subscriber state_sub_;
	bool isHooverRun;



public:
	HooverNode()  {
		isHooverRun = false;
			state_sub_ = nh_.subscribe("/hoover_state", 1, &HooverNode::stateCb, this);

			std::string dev = "/dev/hoover";
			sp = new SerialSwitch(dev);
			number_of_balls = 0;

		}

	~HooverNode() {
	}
	int number_of_balls;

	void stateCb(const std_msgs::Int16& state) {
//		ROS_INFO("stateCb data = %d", state.data);

		int data = state.data;
		if (data == 0 || data == 1){
			if (sp->isConnected()) {
				if (ros::ok()) {
					sp->update(data);
				}
			} else {
				ROS_ERROR("Connection to device  failed");
			}
		}

	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "hoover_node");
	HooverNode hn;
//	ros::spin();

	ros::Rate loop_rate(10);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();

		if (sp->isConnected()) {
			if (ros::ok()) {
				hn.number_of_balls = sp->getNumbeOfBalls();	//	zapytaj o liczbe pileczek
			//	ROS_INFO("number_of_balls = %d", hn.number_of_balls);
			}
		} else {
			ROS_ERROR("Connection to device  failed");
		}

	}
}





