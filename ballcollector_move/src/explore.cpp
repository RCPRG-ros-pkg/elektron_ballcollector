#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <string>


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

#include <move_base_msgs/MoveBaseActionGoal.h>

#include <nav_msgs/GridCells.h>

#include <tf/transform_datatypes.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <actionlib/server/simple_action_server.h>
#include <scheduler/SchedulerAction.h>


#define PI 3.14159265



typedef enum
{
	STOP = 0,
    FULL_ROTATE = 1,
    RANDOM_ROTATE = 2,
    MAX_FORWARD = 3,
}ExploreState;



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


//geometry_msgs::PoseStamped currPose;

geometry_msgs::Pose2D currPose;

costmap_2d::Costmap2DROS* costmap_ros;
costmap_2d::Costmap2D costmap;


bool canMove();

void rotateRand();
void goStraight();
void stop();
double getRandomAngle();
double getRandomLenght();

void obstaclesMapCallback(const nav_msgs::GridCells& msg);



double getAngle(const geometry_msgs::Quaternion& qMsg);
void setAngle(double angle,  geometry_msgs::Quaternion& qMsg);
void publishPose(geometry_msgs::Pose2D pose2d);


bool canMove(float x, float y);
double getCost(float x, float y);




class Explore{

private:
	ros::NodeHandle nh_;
	ros::Subscriber alghoritm_state_sub_;
	ros::Subscriber posSub;
	ros::Subscriber deadlock_service_state_sub;


	actionlib::SimpleActionServer<scheduler::SchedulerAction> as_;
	std::string action_name_;
	scheduler::SchedulerFeedback feedback_;
	scheduler::SchedulerResult result_;

	bool explore_;
	ExploreState explore_state_;

public:
	bool firstGoalSend;
	MoveBaseClient ac_;
	tf::TransformListener tf_listener_;


	Explore(std::string name):
		as_(nh_, name, boost::bind(&Explore::executeCB, this, _1), false),
		action_name_(name),
		ac_("move_base", true)
	{
		as_.start();
		firstGoalSend = false;

		//all_balls = nh_.subscribe < geometry_msgs::PoseArray > ("allBalls", 10, &ChooseAccessibleBalls::allBallsCb, this);

		alghoritm_state_sub_ =   nh_.subscribe("alghoritm_state", 1, &Explore::alghoritmStateCallBack, this);
		posSub = nh_.subscribe("/amcl_pose", 1, &Explore::poseCallback, this);
		deadlock_service_state_sub = nh_.subscribe("/deadlock_service_state", 1, &Explore::deadlockServiceStateCb, this);


		explore_state_ = STOP;
		explore_ = false;
	}
	void publishPose(float x, float y, float theta);
	void stopExplore();
	void alghoritmStateCallBack(const std_msgs::String& msg);
	void deadlockServiceStateCb(const std_msgs::String& state);
	void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
	void transfromRobotToOdomPosition(float x_robot, float y_robot, float &x_odom, float &y_odom);
	void transfromRobotToMapPosition(float x_robot, float y_robot, float &x_map, float &y_map);

	void transFromOdomToMapPosition(float x_odom_pose, float y_odom_pose, float theta,
			float &x_map_pose, float &y_map_pose, tf::Quaternion& q);

	void getRobotPositionInOdom(float &x_odom_pose, float &y_odom_pose);

	float getRobotAngleInMap();
	float getRobotAngleInOdom();
	float addPiToAngle(float angle);

	void randomRotate();
	void randomForward();
	void robotFullRotate();
	void maxForward();

	void executeCB(const scheduler::SchedulerGoalConstPtr &goal);
	bool canExplore(){ return explore_;};
	void setCanExplore(bool explore){explore_ = explore;};
	ExploreState getExploreState(){ return explore_state_;};
	void setExploreState(ExploreState explore_state){explore_state_ = explore_state;};
	bool isCurrentGoalDone();
};



int main(int argc, char** argv) {

	ros::init(argc, argv, "robot_move");


	Explore robot_explore(ros::this_node::getName());

//	ros::Subscriber obsMapSub = nh_.subscribe("/move_base/local_costmap/inflated_obstacles", 1, obstaclesMapCallback);


	costmap_ros = new costmap_2d::Costmap2DROS("local_costmap", robot_explore.tf_listener_);
	//costmap_ros->getCostmapCopy(costmap);

	double infRad = costmap_ros->getInflationRadius();
	ROS_INFO("infRad =%f",infRad);

	ros::Rate loop_rate(1);

//	float rand_theta;
//	double x_odom, y_odom, rand_x, rand_y, x_map, y_map;


	 while (!robot_explore.ac_.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server to come up");
	 }


	while (ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();

		if(robot_explore.canExplore() == false){
	//		ROS_INFO("Robot move - no explore - start ...");
	//		robot_explore.stopExplore();
	//		ros::Duration(1.0).sleep();
	//		ROS_INFO("Robot move - no explore - stop");
			continue;
		}
		else{

			costmap_ros->getCostmapCopy(costmap);

			if(robot_explore.getExploreState() == STOP){
				if(robot_explore.canExplore()){
					robot_explore.setExploreState(FULL_ROTATE);
					robot_explore.robotFullRotate();
					ROS_INFO("Start Explore go to FULL_ROTATE state");
				}
			}
			else if(robot_explore.getExploreState() == FULL_ROTATE){

				if(robot_explore.isCurrentGoalDone()){
					robot_explore.setExploreState(RANDOM_ROTATE);
					robot_explore.randomRotate();
					ROS_INFO("go to RANDOM_ROTATE state");
				}

			}
			else if(robot_explore.getExploreState() == RANDOM_ROTATE){

				if(robot_explore.isCurrentGoalDone()){
					robot_explore.setExploreState(MAX_FORWARD);
					robot_explore.maxForward();
					ROS_INFO("go to MAX_FORWARD state");
				}
			}
			else if(robot_explore.getExploreState() == MAX_FORWARD){

				if(robot_explore.isCurrentGoalDone()){
					robot_explore.setExploreState(RANDOM_ROTATE);
					robot_explore.randomRotate();
					ROS_INFO("go to RANDOM_ROTATE state");
				}
			}
		}






		/*

		robot_explore.robotFullRotate();




		while(explore == true){

			robot_explore.randomRotate();
			std::cout<<"state randomRotate = "<<robot_explore.ac_.getState().toString()<<std::endl;

			if (robot_explore.ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				ROS_INFO("Rotate  Hooray, the base moved");
			} else {
				ROS_INFO("Rotate  The base failed to move for some reason");
			}

			//robotMove.randomForward();
			robot_explore.maxForward();
			std::cout<<"state maxForward = "<<robot_explore.ac_.getState().toString()<<std::endl;
			if (robot_explore.ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				ROS_INFO("Forward  Hooray, the base moved");
			} else {
				ROS_INFO("Forward  The base failed to move for some reason");
			}
		//	std::cout<<"1 explore = "<<explore<<std::endl;
			ros::spinOnce();
			loop_rate.sleep();
		//	std::cout<<"2 explore = "<<explore<<std::endl;

		}

		*/


	}


	return 0;
}


void Explore::stopExplore(){

	  ac_.cancelAllGoals ();
}

bool canMove(){
	return true;
}


void rotateRand(){

}


double getRandomAngle(){
	return 180.0;
}

double getRandomLenght(){
	return 0.3;
}


void Explore::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
 // ROS_INFO("I heard pose (x, y) = (%f, %f)", msg->pose.pose.position.x, msg->pose.pose.position.y);

	//	aktualizacja pozycji robota

	currPose.x = msg->pose.pose.position.x;
	currPose.y = msg->pose.pose.position.y;
	currPose.theta = getAngle(msg->pose.pose.orientation);

//	ROS_INFO("currPose.theta= %f", currPose.theta);

}

void obstaclesMapCallback(const nav_msgs::GridCells& msg){
//	ROS_INFO("obstaclesMapCallback");
}

void Explore::alghoritmStateCallBack(const std_msgs::String& msg){

	/*
	ROS_INFO("alghoritmStateCallBack ");
	std::cout<<msg.data<<std::endl;
	if(msg.data.compare("SEARCH_BALLS") ==0 ){
		ROS_INFO("explore == true");

		explore = true;
	}else if(msg.data.compare("GO_TO_BALL") == 0 ){
		ROS_INFO("explore == false");

		if(explore == true){
			//	pierwsza zmiana
			stopExplore();
		}

		explore = false;
	}*/
}

void Explore::deadlockServiceStateCb(const std_msgs::String& state){

	/*
	ROS_INFO("deadlockServiceStateCb ");
	if(state.data.compare("RUNNING") ==0 ){
		ROS_INFO("explore == true");
		explore = true;
	}
	else if(state.data.compare("NOT_RUNNING") == 0 ){
		ROS_INFO("explore == false");
		if(explore == true){
			//	pierwsza zmiana
			stopExplore();
		}
		explore = false;
	}

	*/
}



double getAngle(const geometry_msgs::Quaternion& qMsg){

	  double unused, yaw ;
	  btQuaternion q_pose;

	  tf::quaternionMsgToTF(qMsg, q_pose);
	  btMatrix3x3 m_pose(q_pose);
	  m_pose.getRPY(unused, unused, yaw);
	 // yaw = yaw * 180 / PI;
	  return yaw;
}



void setAngle(double angle,  geometry_msgs::Quaternion& qMsg){

	btQuaternion q_result;
	q_result.setRPY(.0, .0, angle);
	tf::quaternionTFToMsg(q_result, qMsg);

}


void publishPose(geometry_msgs::Pose2D pose2d ){
/*
	  move_base_msgs::MoveBaseGoal goal;

	  goal.target_pose.pose.position.x = x;
	  goal.target_pose.pose.position.y = y;

	  goal.target_pose.pose.orientation = qMsg;

	  goal.target_pose.header.stamp = ros::Time::now();
	  goal.target_pose.header.frame_id ="/map";


	  ROS_INFO("Sending goal");
	  ac_.sendGoal(goal);

	  */

}

void Explore::publishPose(float x, float y, float theta){

	  move_base_msgs::MoveBaseGoal goal;

	  goal.target_pose.pose.position.x = x;
	  goal.target_pose.pose.position.y = y;

	  geometry_msgs::Quaternion qMsg;
	  setAngle(theta, qMsg);

	  goal.target_pose.pose.orientation = qMsg;

	  goal.target_pose.header.stamp = ros::Time::now();
	  goal.target_pose.header.frame_id ="/map";


	  ROS_INFO("Sending goal...");
	  ac_.sendGoal(goal);

	  firstGoalSend = true;

}

float Explore::addPiToAngle(float angle){
	angle += PI;								//	kąt od 0 do 2PI
	angle += PI;								//	kąt od 0 do 2PI przekręcony o PI

	if(angle > 2*PI){
		angle = angle - 2*PI;
	}
	angle -= PI;
	return angle;
}

void Explore::robotFullRotate(){

	ROS_INFO("enter robotFullRotate ");
	float robot_odom_x, robot_odom_y;
	getRobotPositionInOdom(robot_odom_x, robot_odom_y);


	float angle_in_odom = getRobotAngleInOdom();		//	kat od -PI do + PI
	float anglePlusPI = addPiToAngle(angle_in_odom);

	tf::Quaternion q;
	float goal_map_x, goal_map_y;
	transFromOdomToMapPosition(robot_odom_x, robot_odom_y, anglePlusPI, goal_map_x, goal_map_y, q);

	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(q, qMsg);


	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.pose.position.x = goal_map_x;
	goal.target_pose.pose.position.y = goal_map_y;
	goal.target_pose.pose.position.z = 0;


	goal.target_pose.pose.orientation = qMsg;

	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id ="/map";

//	ROS_INFO("Sending goal 1");
	ac_.sendGoal(goal);

	/*

	ROS_INFO("wait for result");
	ac_.waitForResult();


	getRobotPositionInOdom(robot_odom_x, robot_odom_y);


	angle_in_odom = getRobotAngleInOdom();		//	kat od -PI do + PI
	anglePlusPI = addPiToAngle(angle_in_odom);

	transFromOdomToMapPosition(robot_odom_x, robot_odom_y, anglePlusPI, goal_map_x, goal_map_y, q);

	tf::quaternionTFToMsg(q, qMsg);


	goal.target_pose.pose.position.x = goal_map_x;
	goal.target_pose.pose.position.y = goal_map_y;
	goal.target_pose.pose.position.z = 0;


	goal.target_pose.pose.orientation = qMsg;

	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id ="/map";

	ROS_INFO("Sending goal 2");
	ac_.sendGoal(goal);

	ROS_INFO("wait for result");
	ac_.waitForResult();


*/
	ROS_INFO("leave robotFullRotate");
}

void Explore::randomRotate(){

	ROS_INFO("enter randomRotate ");
	float robot_odom_x, robot_odom_y;
	getRobotPositionInOdom(robot_odom_x, robot_odom_y);


	float angle = (((rand() % 360) - 180) * PI) / 180.0;

	tf::Quaternion q;
	float goal_map_x, goal_map_y;
	transFromOdomToMapPosition(robot_odom_x, robot_odom_y, angle, goal_map_x, goal_map_y, q);

	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(q, qMsg);


	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.pose.position.x = goal_map_x;
	goal.target_pose.pose.position.y = goal_map_y;
	goal.target_pose.pose.position.z = 0;


	goal.target_pose.pose.orientation = qMsg;

	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id ="/map";

//	ROS_INFO("Sending goal");
	ac_.sendGoal(goal);

//	ROS_INFO("wait for result");
//	ac_.waitForResult();
	ROS_INFO("leave randomRotate");



}



void Explore::randomForward(){

	ROS_INFO("enter randomForward ");

	int counter = 0;

	float rand_x, rand_y, x_odom, y_odom, x_map, y_map;
	do {
		rand_x = (rand() % 10) / 20.0;
		rand_y = 0;
		transfromRobotToOdomPosition(rand_x, rand_y, x_odom, y_odom);
		++counter;
		if(counter > 10){
			ROS_INFO("leave randomForward, can not move forward");
			return;
		}

	} while (!canMove(x_odom, y_odom));


	float robotAngleInMap = getRobotAngleInMap();

	transfromRobotToMapPosition(rand_x, rand_y, x_map, y_map);
//	ROS_INFO("robot move to (%f,%f)", x_map, y_map);

	publishPose(x_map, y_map, robotAngleInMap);

//	ROS_INFO("wait for result");
//	ac_.waitForResult();

	ROS_INFO("leave randomForward");
}



void Explore::maxForward(){
	ROS_INFO("enter maxForward ");

	int counter = 0;

	float d_x = 0, d_y=0, x_map, y_map, x_odom, y_odom;
	transfromRobotToOdomPosition(d_x, d_y, x_odom, y_odom);
	 while (canMove(x_odom, y_odom)){
		d_x += 0.1;
		d_y = 0;
		transfromRobotToOdomPosition(d_x, d_y, x_odom, y_odom);
	/*	++counter;
		if(counter > 10){
			ROS_INFO("leave maxForward, can not move forward");
			return;
		}*/

	};

//	ROS_INFO("d_x = %f", d_x);

	float robotAngleInMap = getRobotAngleInMap();

	transfromRobotToMapPosition(d_x, d_y, x_map, y_map);
//	ROS_INFO("robot move to (%f,%f)", x_map, y_map);

	publishPose(x_map, y_map, robotAngleInMap);

//	ROS_INFO("wait for result");
//	ac_.waitForResult();

	ROS_INFO("leave maxForward");
}



void Explore::getRobotPositionInOdom(float &x_odom_pose, float &y_odom_pose){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/odom", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/base_link", now,  tfOR);

	x_odom_pose = tfOR.getOrigin ()[0];				//	wspolrzedne robota w ukladzie odom
	y_odom_pose = tfOR.getOrigin ()[1];				//	wspolrzedne robota w ukladzie odom

}


float Explore::getRobotAngleInMap(){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/map", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/map", "/base_link", now,  tfOR);

//	x_odom_pose = tfOR.getOrigin ()[0];				//	wspolrzedne robota w ukladzie odom
//	y_odom_pose = tfOR.getOrigin ()[1];				//	wspolrzedne robota w ukladzie odom

	return tf::getYaw(tfOR.getRotation());

}


float Explore::getRobotAngleInOdom(){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/odom", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/base_link", now,  tfOR);

//	x_odom_pose = tfOR.getOrigin ()[0];				//	wspolrzedne robota w ukladzie odom
//	y_odom_pose = tfOR.getOrigin ()[1];				//	wspolrzedne robota w ukladzie odom

	return tf::getYaw(tfOR.getRotation());

}


bool canMove(float x, float y){
	  // Get a copy of the current costmap to test. (threadsafe)
	  costmap_2d::Costmap2D costmap;
	  costmap_ros->getCostmapCopy( costmap );

	  // Coordinate transform.
	  unsigned int cell_x, cell_y;
	  if( !costmap.worldToMap( x, y, cell_x, cell_y )){
	 //   res.cost = -1.0;
	    return false;
	  }

	  double cost = double( costmap.getCost( cell_x, cell_y ));
	  ROS_INFO("cost = %f", cost);
	  if(cost <= 1){
		  return true;
	  }
	  else{
		  return false;
	  }
	 //ROS_INFO(" world pose = (%f, %f)   map pose = (%d, %d)  cost =%f", x, y, cell_x, cell_y,  cost);
}

double getCost(float x, float y){
  // Get a copy of the current costmap to test. (threadsafe)
  costmap_2d::Costmap2D costmap;
  costmap_ros->getCostmapCopy( costmap );

  // Coordinate transform.
  unsigned int cell_x, cell_y;
  if( !costmap.worldToMap( x, y, cell_x, cell_y )){
	  return -1.0;
  }

  double cost = double( costmap.getCost( cell_x, cell_y ));
//  ROS_INFO(" world pose = (%f, %f)   map pose = (%d, %d)  cost =%f", x, y, cell_x, cell_y,  cost);
  return cost;
}


void Explore::transfromRobotToOdomPosition(float x_robot, float y_robot, float &x_odom, float &y_odom){

// 	ROS_INFO(" enter transfromRobotToOdomPosition  robot pose (%f, %f)", x_robot, y_robot);


	ros::Time now = ros::Time::now();

//	ROS_INFO("1");
	tf::StampedTransform tfRO;
//	ROS_INFO("2");
	tf_listener_.waitForTransform("/base_link", "/odom", now, ros::Duration(1.0));
//	ROS_INFO("3");
	tf_listener_.lookupTransform ("/base_link", "/odom",  now,  tfRO);
//	ROS_INFO("4");
	tf::Transform tfOR = tfRO.inverse();
//	ROS_INFO("5");
	float noused = 0.0;
//	ROS_INFO("6");
	tf::Transform tfRD;
//	ROS_INFO("7");
	tfRD.setOrigin(tf::Vector3(x_robot, y_robot, noused));
//	ROS_INFO("8");
	tf::Transform tfOD = tfOR * tfRD;


	 x_odom = tfOD.getOrigin ()[0];				//	wspolrzedna x_robot w ukladzie odom
	 y_odom = tfOD.getOrigin ()[1];				//	wspolrzedna y_robot w ukladzie odom

//	 ROS_INFO("x_odom = %f,  y_odom = %f", x_odom, y_odom);
}

void Explore::transfromRobotToMapPosition(float x_robot, float y_robot, float &x_map, float &y_map){

	ros::Time now = ros::Time::now();

	tf::StampedTransform tfRO;
	tf_listener_.waitForTransform("/base_link", "/odom", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/base_link", "/odom", now,  tfRO);
	tf::Transform tfOR = tfRO.inverse();


	tf::StampedTransform tfOM;
	tf_listener_.waitForTransform("/odom", "/map", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/map", now,  tfOM);
	tf::Transform tfMO = tfOM.inverse();


	tf::Transform tfRM = tfMO*tfOR;

	float noused = 0.0;

	tf::Transform tfRD;
	tfRD.setOrigin(tf::Vector3(x_robot, y_robot, noused));

	tf::Transform tfOD = tfOR * tfRD;


	tf::Transform tfMD = tfMO * tfOD;


	x_map = tfMD.getOrigin ()[0];				//	wspolrzedne docelowe w ukladzie mapy
	y_map = tfMD.getOrigin ()[1];				//	wspolrzedne docelowe w ukladzie mapy


}


void Explore::transFromOdomToMapPosition(float x_odom_pose, float y_odom_pose, float theta,
		float &x_map_pose, float &y_map_pose, tf::Quaternion& q){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfMO;								//	odom w map
	tf_listener_.waitForTransform("/map", "/odom", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/map", "/odom", now,  tfMO);


	tf::Transform tfOD;

	tf::Quaternion quat;
	quat.setRPY(0, 0, theta);

	tfOD.setOrigin(tf::Vector3(x_odom_pose, y_odom_pose, 0.0));			//	docelowe w odom
	tfOD.setRotation( quat );

	tf::Transform tfMD = tfMO * tfOD;							//	docelowe w map


	x_map_pose = tfMD.getOrigin ()[0];				//	wspolrzedne docelowe w ukladzie map
	y_map_pose = tfMD.getOrigin ()[1];				//	wspolrzedne docelowe w ukladzie map
	q = tfMD.getRotation();
}


bool Explore::isCurrentGoalDone(){
	bool is_done = ac_.getState().isDone();
	return is_done;
}



//	metoda jest uruchamiana w osobnym watku i nie blokuje calbackow
void Explore::executeCB(const scheduler::SchedulerGoalConstPtr &goal){
	ROS_INFO("enter executeCB, goal = %i", goal->value);

	if(goal->value == 1){
		explore_ = true;

	}
	else if(goal->value == 0){
		//	koniec ekslporacji
		stopExplore();
		explore_state_ = STOP;
		explore_ = false;
	}

	feedback_.value = 0;

	as_.publishFeedback(feedback_);
	result_.value = feedback_.value;

	as_.setSucceeded(result_);
	ROS_INFO("leave executeCB");

}









