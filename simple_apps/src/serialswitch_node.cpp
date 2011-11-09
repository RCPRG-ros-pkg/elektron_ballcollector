#include <ros/ros.h>
#include <std_msgs/Int16.h>

#include "serialswitch.hpp"



//ros::Time cmd_time;

SerialSwitch *sp;


/*
void joyCallback(const joy::Joy::ConstPtr& joy)
{
      
     sp->setState(13);
      
      
}
*/


int main(int argc, char** argv){
    ros::init(argc, argv, "serialswitch_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    
    ros::Publisher serialswitch_pub = n.advertise<std_msgs::Int16>("state",1);
    
 //   ros::Subscriber joy_sub = n.subscribe("joy", 10, &joyCallback)
    
   	ros::Rate loop_rate(1);
    
    	std::string dev;
      	bool dump;

	if (!nh.getParam("device", dev)) {
		 dev = "/dev/ttyUSB1"; 
	}

	if (!nh.getParam("dump", dump)) {
	dump = false;
	}

    	std_msgs::Int16 state;
    	state.data = 12;
    
 	sp = new SerialSwitch(dev);
        
        if (sp->isConnected()) {
		if (dump)
			sp->dump();

	    
		while(ros::ok()){
		  //  ros::Time current_time = ros::Time::now();

			sp->update();

			serialswitch_pub.publish(state);

			ros::spinOnce();
			loop_rate.sleep();
		}
	} else {
		ROS_ERROR("Connection to device %s failed", dev.c_str());
	}


    
    return 0;
}       
