#include <ros/ros.h>
#include <std_msgs/Int16.h>

#include "serialswitch.hpp"


SerialSwitch *sp;

class HooverNode {

	ros::NodeHandle nh_;
	ros::Subscriber state_sub_;


public:
	HooverNode()  {
			state_sub_ = nh_.subscribe("/state", 1, &HooverNode::stateCb, this);

			std::string dev = "/dev/ttyUSB1";
			sp = new SerialSwitch(dev);

		}

	~HooverNode() {
	}



	void stateCb(const std_msgs::Int16& state) {
		ROS_INFO("stateCb");
		if (sp->isConnected()) {
			if(ros::ok()){
				sp->update();
			}
		} else {
			ROS_ERROR("Connection to device  failed");
		}
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "hoover_node");
	HooverNode hn;
	ros::spin();
	return 0;
}






