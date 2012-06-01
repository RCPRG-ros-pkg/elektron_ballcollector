#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>



#include "TracedBall.h"



class ChooseOneBall {

	ros::NodeHandle nh_;
	ros::Subscriber all_balls;
	ros::Publisher goal_pub_;
	ros::Publisher selected_ball_pub_;
	ros::Publisher selected_ball_marker_pub_;
	tf::TransformListener tf_listener_;
	bool first;
	std::vector<TracedBall> traced_balls_;
	int isBallAlreadyTraced(TracedBall& tBall);
	void downAllBallsHits();
	void showAllTracedBalls();
	void showActiveBalls();
	void stopTraceBall(int id);
	void updateTracedBalls();
	void unselectAllBalls();
	void selectBall(int id);
	int getSelectedBallId();


	void getRobotPositionInOdom(float &x_odom_pose, float &y_odom_pose);
	float getDistanceFromBallToRobot(int id);
	int getClosestBallId();

	TracedBall getTracedBall(int id);
	void publishSelectedBall();




public:


	ChooseOneBall(){

		all_balls = nh_.subscribe < geometry_msgs::PoseArray > ("allBalls", 10, &ChooseOneBall::allBallsCb, this);
		selected_ball_pub_ =  nh_.advertise < geometry_msgs::Point > ("one_selected_ball", 1);
		selected_ball_marker_pub_ = nh_.advertise < visualization_msgs::Marker > ("selected_ball_marker", 1);
	}

	~ChooseOneBall() {
	}
	void allBallsCb(const geometry_msgs::PoseArrayConstPtr& all_balls_msg);

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "chooseOneBall");

	ROS_INFO("init  ChooseOneBall");
	ChooseOneBall chooseOneBall;

	ros::spin();
	return 0;

}

void ChooseOneBall::allBallsCb(const geometry_msgs::PoseArrayConstPtr& all_balls_msg){
//	ROS_INFO("enter allBallsCb");

//	traced_balls_.clear();

	//	obniza hits wszystkich sledzonych pilek
	downAllBallsHits();


	std::vector<geometry_msgs::Pose> allBalls = all_balls_msg->poses;
//	ROS_INFO("number of balls = %d", (int)allBalls.size());

	for (unsigned int i = 0; i < allBalls.size(); i++){

		geometry_msgs::Pose pose;

		float x = allBalls[i].position.x;
		float y = allBalls[i].position.y;
//		float z = allBalls[i].position.z;

	//	ROS_INFO("(x, y, z) = (%f, %f, %f)", x, y, z);
	//	ROS_INFO("z  =  %f", z);

		TracedBall tBall(x, y);

		int tracedBallId = isBallAlreadyTraced(tBall);
		if(tracedBallId != -1){
			//	jesli pilka jest juz sledzona, to podnosimy jej hits
			traced_balls_[tracedBallId].hitsUp();
			traced_balls_[tracedBallId].updateXY(x, y);
		}
		else{
			//	jesli jeszcze nie jest sledzona, to zaczynamy ja sledzic
			traced_balls_.push_back(tBall);
		}

	}

	updateTracedBalls();
//	showActiveBalls();

	publishSelectedBall();



//	ROS_INFO("traced balls %d", (int)traced_balls_.size());
//	std::cout<<std::endl<<std::endl;

}

int ChooseOneBall::isBallAlreadyTraced(TracedBall& tBall){

	for (unsigned int i = 0; i < traced_balls_.size(); i++){

		if(traced_balls_[i].isTheSameBall(tBall) == true){
			return i;
		}
	}
	return -1;
}

void ChooseOneBall::downAllBallsHits(){
	for (unsigned int i = 0; i < traced_balls_.size(); i++){
		traced_balls_[i].hitsDown();
	}
}


void ChooseOneBall::showAllTracedBalls(){
	std::cout<<"All traced balls"<<std::endl;
	for (unsigned int i = 0; i < traced_balls_.size(); i++){
		std::cout<<traced_balls_[i].toString()<<std::endl;
	}
}

void ChooseOneBall::showActiveBalls(){
	std::cout<<"All active traced balls"<<std::endl;
	for (unsigned int i = 0; i < traced_balls_.size(); i++){
		if(traced_balls_[i].isBall() == true){
			std::cout<<traced_balls_[i].toString()<<std::endl;
		}
	}
}

void ChooseOneBall::stopTraceBall(int id){
	traced_balls_.erase(traced_balls_.begin() + id);
}

void ChooseOneBall::updateTracedBalls(){


	std::vector<TracedBall>::iterator i = traced_balls_.begin();
	while (i != traced_balls_.end())
	{
	    bool toRemove = (*i).isMistake();
	    if (toRemove)
	    {
//	    	ROS_INFO("remove");
	    	//traced_balls_.erase(i++);  // alternatively,
	    	i = traced_balls_.erase(i);
	    }
	    else
	    {
	        ++i;
	    }
	}
	int selectedBallId = getSelectedBallId();
	if(selectedBallId == -1){
		// nie ma zadnej wybranej pilki, trzeba wybrac nowa
		int newSelectedId =  getClosestBallId();
		if(newSelectedId == -1){
			return;
		}
		selectBall(newSelectedId);
	}
	else{
		//	jest wybrana pilka, nic nie robimy
	}
}


void ChooseOneBall::unselectAllBalls(){
	for (unsigned int i = 0; i < traced_balls_.size(); i++){
		traced_balls_[i].unselect();
	}
}
void ChooseOneBall::selectBall(int id){
	ROS_INFO("CHANGE SELECTED BALL");
	unselectAllBalls();
	traced_balls_[id].select();
}
int ChooseOneBall::getSelectedBallId(){
	for (unsigned int i = 0; i < traced_balls_.size(); i++){
		if( traced_balls_[i].isSelected() == true){
			return i;
		}
	}
	return -1;
}


void ChooseOneBall::getRobotPositionInOdom(float &x_odom_pose, float &y_odom_pose){


	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/odom", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/base_link", now,  tfOR);

	x_odom_pose = tfOR.getOrigin ()[0];				//	wspolrzedne robota w ukladzie odom
	y_odom_pose = tfOR.getOrigin ()[1];				//	wspolrzedne robota w ukladzie odom

}




float ChooseOneBall::getDistanceFromBallToRobot(int id){

	float x_odom_robot_pose, y_odom_robot_pose;
	float x_odom_ball_pose = traced_balls_[id].getX();
	float y_odom_ball_pose = traced_balls_[id].getY();

	getRobotPositionInOdom(x_odom_robot_pose, y_odom_robot_pose);


	float dist = sqrt(
			pow(x_odom_ball_pose-x_odom_robot_pose, 2) +
			pow(y_odom_ball_pose-y_odom_robot_pose, 2) );

	return dist;

}


int ChooseOneBall::getClosestBallId(){

	float min_dist = 9999.99;
	int closest_ball_id = -1;

	for (unsigned int i = 0; i < traced_balls_.size(); i++){
		float curr_dist = getDistanceFromBallToRobot(i);

		if(curr_dist <  min_dist){
			min_dist = curr_dist;
			closest_ball_id = i;
		}
	}

	return closest_ball_id;
}

TracedBall ChooseOneBall::getTracedBall(int id){
	return traced_balls_[id];
}


void ChooseOneBall::publishSelectedBall() {

	int selectedBallId = getSelectedBallId();
	if(selectedBallId == -1){

		geometry_msgs::Point pose;
		pose.x = 1000;
		pose.y = 1000;
		pose.z = 1000;
		selected_ball_pub_.publish(pose);

		return;
	}



	TracedBall tracedBall = getTracedBall(selectedBallId);

	geometry_msgs::Point pose;

	pose.x = tracedBall.getX();
	pose.y = tracedBall.getY();
	pose.z = 0;

	selected_ball_pub_.publish(pose);

	visualization_msgs::Marker marker;
	marker.header.frame_id = "/odom";
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
//	marker.pose.x = tracedBall.getX();
//	marker.pose.y = tracedBall.getY();
//	marker.pose.z = 0;
	marker.pose.position = pose;


	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;


	selected_ball_marker_pub_.publish(marker);

}


