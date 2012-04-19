#include <stdlib.h>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int16.h>

#include <move_base_msgs/MoveBaseActionGoal.h>

#include <nav_msgs/GridCells.h>

#include <tf/transform_datatypes.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>


#define PI 3.141592651111


ros::Publisher vel_pub_;
ros::Publisher goal_pub_;
//geometry_msgs::PoseStamped currPose;

geometry_msgs::Pose2D currPose;

costmap_2d::Costmap2DROS* costmap_ros;
costmap_2d::Costmap2D costmap;

bool moveRandom = true;



bool canMove();

void rotateRand();
void goStraight();
void stop();
double getRandomAngle();
double getRandomLenght();

void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
void obstaclesMapCallback(const nav_msgs::GridCells& msg);
void alghoritmStateCallBack(const std_msgs::Int16& msg);


double getAngle(const geometry_msgs::Quaternion& qMsg);
void setAngle(double angle,  geometry_msgs::Quaternion& qMsg);
void publishPose(geometry_msgs::Pose2D pose2d);
void publishPose(float x, float y, float theta);

bool canMove(float x, float y);
double getCost(float x, float y);

void transfromRobotToOdomPosition(double x_robot, double y_robot, double &x_odom, double &y_odom);

void transfromRobotToMapPosition(double x_robot, double y_robot, double &x_map, double &y_map);


int main(int argc, char** argv) {

ROS_INFO("Start");
	ros::init(argc, argv, "robot_move");

	ros::NodeHandle nh_;

	vel_pub_ = nh_.advertise < geometry_msgs::Twist > ("cmd_vel", 1);
	goal_pub_ = nh_.advertise < move_base_msgs::MoveBaseActionGoal > ("/move_base/goal", 1);


	ros::Subscriber posSub = nh_.subscribe("/amcl_pose", 1, poseCallback);
	ros::Subscriber obsMapSub = nh_.subscribe("/move_base/local_costmap/inflated_obstacles", 1, obstaclesMapCallback);

//	ros::Subscriber alghoritm_state_sub_ =   nh_.subscribe("alghoritm_state", 1, alghoritmStateCallBack);

	ROS_INFO("1");

	tf::TransformListener tf_listener;
	costmap_ros = new costmap_2d::Costmap2DROS("local_costmap", tf_listener);
	//costmap_ros->getCostmapCopy(costmap);

	double infRad = costmap_ros->getInflationRadius();
	ROS_INFO("infRad =%f",infRad);


	ros::Rate loop_rate(1);

	int count = 0;

	float rand_theta, rand_dx, rand_dy;
	double x_odom, y_odom, rand_x, rand_y, x_map, y_map;

	while (ros::ok()) {

		if(!moveRandom){
			transfromRobotToMapPosition(0, 0, x_map, y_map);
			ROS_INFO("robot stop");
			publishPose(x_map, y_map, currPose.theta);
			continue;
		}

		costmap_ros->getCostmapCopy(costmap);

		ROS_INFO("counter =%d ", count);

		ros::spinOnce();

		loop_rate.sleep();
		++count;

		// co 5 cykli szukamy nowej pozycji
		if(count > 5){
			count = 0;

			do{
				rand_theta = (((rand()%360) -180)*PI)/180.0;
				rand_x = (rand()%10)/5.0;
				rand_y = (rand()%10)/5.0;

				transfromRobotToOdomPosition(rand_x, rand_y, x_odom, y_odom);

			}while(!canMove(x_odom, y_odom));

			transfromRobotToMapPosition(rand_x, rand_y, x_map, y_map);
			ROS_INFO("robot move to (%f,%f)", x_map, y_map);
			publishPose(x_map, y_map, rand_theta);

		}
	}
	return 0;
}


bool canMove(){
	return true;
}


void rotateRand(){

}

void goStraight(){

	geometry_msgs::Twist vel;
	vel.angular.z = 0.0;
	vel.linear.x = 0.1;
	vel_pub_.publish(vel);
}

void stop(){

	geometry_msgs::Twist vel;
	vel.angular.z = 0.0;
	vel.linear.x = 0.0;
	vel_pub_.publish(vel);
}


double getRandomAngle(){
	return 180.0;
}

double getRandomLenght(){
	return 0.3;
}


void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
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

void alghoritmStateCallBack(const std_msgs::Int16& msg){
	ROS_INFO("alghoritmStateCallBack, data = %d", msg.data);
	if(msg.data == 0){
		moveRandom = true;
	}else{
		moveRandom = false;
	}
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


void publishPose(geometry_msgs::Pose2D pose2d){

	ROS_INFO("enter publishPose, x=%f, y=%f, theta=%f", pose2d.x, pose2d.y, pose2d.theta);

	move_base_msgs::MoveBaseActionGoal goal;

	goal.goal.target_pose.pose.position.x = pose2d.x;
	goal.goal.target_pose.pose.position.y = pose2d.y;
	goal.goal.target_pose.pose.position.z = 0;

	setAngle(pose2d.theta, goal.goal.target_pose.pose.orientation);

	goal.goal.target_pose.header.frame_id ="/map";

	goal_pub_.publish(goal);

}

void publishPose(float x, float y, float theta){

	ROS_INFO("enter publishPose, x=%f, y=%f, theta=%f", x, y, theta);
	//geometry_msgs::PoseStamped poseToPublish;

	move_base_msgs::MoveBaseActionGoal goal;

	goal.goal.target_pose.pose.position.x = x;
	goal.goal.target_pose.pose.position.y = y;
	goal.goal.target_pose.pose.position.z = 0;

	setAngle(theta, goal.goal.target_pose.pose.orientation);

	goal.goal.target_pose.header.frame_id ="/map";

	goal_pub_.publish(goal);

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
	  if(cost <= 127){
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
  ROS_INFO(" world pose = (%f, %f)   map pose = (%d, %d)  cost =%f", x, y, cell_x, cell_y,  cost);
  return cost;
}


void transfromRobotToOdomPosition(double x_robot, double y_robot, double &x_odom, double &y_odom){

 	ROS_INFO(" enter transfromRobotToOdomPosition  robot pose (%f, %f)", x_robot, y_robot);

	tf::TransformListener tf_listener;

	ros::Time now = ros::Time::now();

	ROS_INFO("1");
	tf::StampedTransform tfRO;
	ROS_INFO("2");
	tf_listener.waitForTransform("/base_link", "/odom", now, ros::Duration(1.0));
	ROS_INFO("3");
	tf_listener.lookupTransform ("/base_link", "/odom",  now,  tfRO);
	ROS_INFO("4");
	tf::Transform tfOR = tfRO.inverse();
	ROS_INFO("5");
	float noused = 0.0;
	ROS_INFO("6");
	tf::Transform tfRD;
	ROS_INFO("7");
	tfRD.setOrigin(tf::Vector3(x_robot, y_robot, noused));
	ROS_INFO("8");
	tf::Transform tfOD = tfOR * tfRD;


	 x_odom = tfOD.getOrigin ()[0];				//	wspolrzedna x_robot w ukladzie odom
	 y_odom = tfOD.getOrigin ()[1];				//	wspolrzedna y_robot w ukladzie odom

}

void transfromRobotToMapPosition(double x_robot, double y_robot, double &x_map, double &y_map){


	tf::TransformListener tf_listener;

	ros::Time now = ros::Time::now();

	tf::StampedTransform tfRO;
	tf_listener.waitForTransform("/base_link", "/odom", now, ros::Duration(1.0));
	tf_listener.lookupTransform ("/base_link", "/odom", now,  tfRO);
	tf::Transform tfOR = tfRO.inverse();


	tf::StampedTransform tfOM;
	tf_listener.waitForTransform("/odom", "/map", now, ros::Duration(1.0));
	tf_listener.lookupTransform ("/odom", "/map", now,  tfOM);
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



/*
ros::Time now = ros::Time::now();

tf::StampedTransform tfRO;
tf_listener.waitForTransform("/base_link", "/odom", now, ros::Duration(1.0));
tf_listener.lookupTransform ("/base_link", "/odom", now,  tfRO);
tf::Transform tfOR = tfRO.inverse();



tf::StampedTransform tfOM;
tf_listener.waitForTransform("/odom", "/map", now, ros::Duration(1.0));
tf_listener.lookupTransform ("/odom", "/map", now,  tfOM);
tf::Transform tfMO = tfOM.inverse();


tf::Transform tfRM = tfMO*tfOR;


float rand_dx = (rand()%5)/10.0;
float rand_dy = (rand()%5)/10.0;
float noused = 0.0;

tf::Transform tfRD;
tfRD.setOrigin(tf::Vector3(rand_dx, rand_dy, noused));

tf::Transform tfOD = tfOR * tfRD;


tf::Transform tfMD = tfMO * tfOD;


//		ROS_INFO(" losowe (x, y) = (%f, %f)",rand_dx, rand_dy);

double x_map = tfMD.getOrigin ()[0];				//	wspolrzedne docelowe w ukladzie mapy
double y_map = tfMD.getOrigin ()[1];				//	wspolrzedne docelowe w ukladzie mapy
ROS_INFO("w ukladzie mapy (x, y) = (%f, %f)",x_map, y_map);


double x_odom = tfOD.getOrigin ()[0];				//	wspolrzedne docelowe w ukladzie odom
double y_odom = tfOD.getOrigin ()[1];				//	wspolrzedne docelowe w ukladzie odom


*/






