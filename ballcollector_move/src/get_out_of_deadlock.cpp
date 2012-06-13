#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <math.h>
#include <std_msgs/String.h>

#include <actionlib/server/simple_action_server.h>
#include <scheduler/SchedulerAction.h>
#include <tf/transform_datatypes.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class GetOutOfDeadlock{

	static const unsigned int FRAME_SIZE = 400;

	ros::NodeHandle nh_;
	tf::TransformListener tf_listener_;

	std::list<geometry_msgs::Point> prev_positions_;

	ros::Publisher deadlock_service_state_pub_;
	ros::Publisher cmd_vel_publisher_;

	actionlib::SimpleActionServer<scheduler::SchedulerAction> as_;
	std::string action_name_;
	scheduler::SchedulerFeedback feedback_;
	scheduler::SchedulerResult result_;

	bool run_deadlock_service_;
	bool collect_prev_position_;

	costmap_2d::Costmap2DROS* costmap_ros;
	costmap_2d::Costmap2D costmap;

	MoveBaseClient ac_;


public:
	GetOutOfDeadlock(std::string name):
		as_(nh_, name, boost::bind(&GetOutOfDeadlock::executeCB, this, _1), false),
		action_name_(name),
		ac_("move_base", true)
	{
		as_.start();
		deadlock_service_state_pub_ = nh_.advertise<std_msgs::String> ("/deadlock_service_state",1);
		cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
		run_deadlock_service_ = false;
		collect_prev_position_ = true;

		costmap_ros = new costmap_2d::Costmap2DROS("local_costmap", tf_listener_);
	}

	~GetOutOfDeadlock() {
	}

	geometry_msgs::Point getRobotPositionInOdom();
	void insertCurrentPosition(geometry_msgs::Point position);
	bool isRobotInDeadlock();

	void deadlockService();

	void publishDeadlock();
	void publishNotDeadlock();
	void rotate(float time_s_);
	void startRotate();
	void stopRotate();

	bool isDeadlockServiceRunning() ;

	void executeCB(const scheduler::SchedulerGoalConstPtr &goal);


	bool canGoForward(float dist);
	void goForward(float dist);
	void transfromRobotToOdomPosition(float x_robot, float y_robot, float &x_odom, float &y_odom);
	void transfromRobotToMapPosition(float x_robot, float y_robot, float &x_map, float &y_map);
	bool canMove(float x, float y);
	float getRobotAngleInMap();



	void publishPose(float x, float y, float theta);
	void setAngle(double angle,  geometry_msgs::Quaternion& qMsg);


};



int main(int argc, char** argv) {
	ros::init(argc, argv, "get_out_of_deadlock");



	GetOutOfDeadlock good(ros::this_node::getName());




//	int deadlock_service_counter = 0;
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();

		good.insertCurrentPosition(good.getRobotPositionInOdom());

		bool is_deadlock = good.isRobotInDeadlock();
	//	good.publishDeadlock();

		if(is_deadlock == true){
			good.publishDeadlock();
		}
		else{
			good.publishNotDeadlock();
		}

		if(good.isDeadlockServiceRunning() == true){
	//		ROS_INFO("run deadlock service, counter = %d", deadlock_service_counter);
	//		++deadlock_service_counter;
		}

	}

	return 0;
}

geometry_msgs::Point GetOutOfDeadlock::getRobotPositionInOdom(){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/odom", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/base_link", now,  tfOR);

	geometry_msgs::Point point;
	point.x = tfOR.getOrigin ()[0];				//	wspolrzedne robota w ukladzie odom
	point.y = tfOR.getOrigin ()[1];				//	wspolrzedne robota w ukladzie odom

	return point;

}

void GetOutOfDeadlock::insertCurrentPosition(geometry_msgs::Point position){
//	ROS_INFO("insertCurrentPosition  (%f, %f)", position.x, position.y);
	if(collect_prev_position_ == false){
		return;
	}

	if(prev_positions_.size() < FRAME_SIZE){
		prev_positions_.push_back(position);

	}
	else{
		prev_positions_.pop_front();
		prev_positions_.push_back(position);
	}
}



bool GetOutOfDeadlock::isRobotInDeadlock(){
//	ROS_INFO("prev_positions_ = %d", prev_positions_.size());

	if(prev_positions_.size() < FRAME_SIZE){
		//	zabezpieczenie przed ciaglym wchodzeniem w stan zakleszczenia
		return false;
	}

	float min_x = 999.9, min_y = 999.9, max_x = -999.9, max_y = -999.9;
	for (std::list<geometry_msgs::Point>::iterator it = prev_positions_.begin(); it != prev_positions_.end(); it++){

	    min_x = std::min(min_x, (float)(*it).x);
	    max_x = std::max(max_x, (float)(*it).x);
	    min_y = std::min(min_y, (float)(*it).y);
	    max_y = std::max(max_y, (float)(*it).y);
	}

	float min_max_dist;
	min_max_dist = sqrt(pow(max_x - min_x,2) + pow(max_y - min_y,2));

//	ROS_INFO("min_max_dist = %f", min_max_dist);

	if(min_max_dist > 0.2){
		return false;
	}
	else{
		return true;
	}
}

void GetOutOfDeadlock::deadlockService(){
	publishDeadlock();

	rotate(3.0);
	prev_positions_.clear();

	publishNotDeadlock();
}


void GetOutOfDeadlock::publishDeadlock(){
	std_msgs::String message;
	message.data = "DEADLOCK";
	deadlock_service_state_pub_.publish(message);
}
void GetOutOfDeadlock::publishNotDeadlock(){
	std_msgs::String message;
	message.data = "NOT_DEADLOCK";
	deadlock_service_state_pub_.publish(message);
}


void GetOutOfDeadlock::rotate(float time_s_){

	geometry_msgs::Twist vel;
	vel.angular.z = 0.3;
	vel.linear.x = 0;
	cmd_vel_publisher_.publish(vel);

	ros::Duration(time_s_).sleep();

	vel.angular.z = 0;
	vel.linear.x = 0;
	cmd_vel_publisher_.publish(vel);
}

void GetOutOfDeadlock::startRotate(){

	geometry_msgs::Twist vel;
	vel.angular.z = 0.3;
	vel.linear.x = 0;
	cmd_vel_publisher_.publish(vel);
}

void GetOutOfDeadlock::stopRotate(){

	geometry_msgs::Twist vel;
	vel.angular.z = 0.0;
	vel.linear.x = 0.0;
	cmd_vel_publisher_.publish(vel);
}


bool GetOutOfDeadlock::isDeadlockServiceRunning(){
	return run_deadlock_service_;
}


//	metoda jest uruchamiana w osobnym watku i nie blokuje calbackow
void GetOutOfDeadlock::executeCB(const scheduler::SchedulerGoalConstPtr &goal){
	ROS_INFO("enter executeCB, goal = %i", goal->value);
	collect_prev_position_ = false;	//	przestajemy zbierac dane o poprzednich pozycjach robota
	prev_positions_.clear();		//	czyscimy historie zmian pozycji robota


	run_deadlock_service_ = true;
	ros::Rate r(1);
	// push_back the seeds for the fibonacci sequence
	feedback_.value = 0;

	as_.publishFeedback(feedback_);
	result_.value = feedback_.value;

	startRotate();

	//	obracamy robotem dopoki nie mozemy jechac o 0.7 metra do przodu
	while(!canGoForward(0.7)){
		startRotate();
		r.sleep();	//	deadlock service running
		ROS_INFO("ROTATE LOOKING FOR A DEADLOCK SOLUTION");
	}
	ROS_INFO("FIND !!!");
	stopRotate();
	goForward(0.7);

	ROS_INFO("wait for result");
	ac_.waitForResult();

	if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	      ROS_INFO("DEADLOCK service finish OK");
	}
	else{
		  ROS_INFO("DEADLOCK service finish with ERROR");
	}


	//	koniec obslugi zakleszczenia

	run_deadlock_service_ = false;	//	koniec obslugi zakleszczenia
	as_.setSucceeded(result_);
	collect_prev_position_ = true;	//	zacznij zbierac poprzednie pozycje
	ROS_INFO("leave executeCB");

}





bool GetOutOfDeadlock::canGoForward(float dist){
	ROS_INFO("enter canGoForward ");

	float d_x = dist, d_y = 0, x_odom, y_odom;
	transfromRobotToOdomPosition(d_x, d_y, x_odom, y_odom);

	if(canMove(x_odom, y_odom)){
		ROS_INFO("leave maxForward");
		return true;
	}
	else{
		ROS_INFO("leave maxForward");
		return false;
	}
}



void GetOutOfDeadlock::goForward(float dist){
	ROS_INFO("enter goForward ");


	float d_x = dist, d_y=0, x_map, y_map;

	float robotAngleInMap = getRobotAngleInMap();
	transfromRobotToMapPosition(d_x, d_y, x_map, y_map);

	publishPose(x_map, y_map, robotAngleInMap);

	ROS_INFO("leave goForward");
}


void GetOutOfDeadlock::transfromRobotToOdomPosition(float x_robot, float y_robot, float &x_odom, float &y_odom){


	ros::Time now = ros::Time::now();
	tf::StampedTransform tfRO;
	tf_listener_.waitForTransform("/base_link", "/odom", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/base_link", "/odom",  now,  tfRO);
	tf::Transform tfOR = tfRO.inverse();
	float noused = 0.0;
	tf::Transform tfRD;
	tfRD.setOrigin(tf::Vector3(x_robot, y_robot, noused));
	tf::Transform tfOD = tfOR * tfRD;

	x_odom = tfOD.getOrigin ()[0];				//	wspolrzedna x_robot w ukladzie odom
	y_odom = tfOD.getOrigin ()[1];				//	wspolrzedna y_robot w ukladzie odom
}


void GetOutOfDeadlock::transfromRobotToMapPosition(float x_robot, float y_robot, float &x_map, float &y_map){

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



bool GetOutOfDeadlock::canMove(float x, float y){

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
	  if(cost <= 1){
		  return true;
	  }
	  else{
		  return false;
	  }
	 //ROS_INFO(" world pose = (%f, %f)   map pose = (%d, %d)  cost =%f", x, y, cell_x, cell_y,  cost);
}




float GetOutOfDeadlock::getRobotAngleInMap(){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/map", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/map", "/base_link", now,  tfOR);

	return tf::getYaw(tfOR.getRotation());

}


void GetOutOfDeadlock::publishPose(float x, float y, float theta){

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

}

void GetOutOfDeadlock::setAngle(double angle,  geometry_msgs::Quaternion& qMsg){

	btQuaternion q_result;
	q_result.setRPY(.0, .0, angle);
	tf::quaternionTFToMsg(q_result, qMsg);

}




