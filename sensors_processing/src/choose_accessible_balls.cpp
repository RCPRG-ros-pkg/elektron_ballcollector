#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>


#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <std_msgs/Int16.h>

#include <nav_msgs/GridCells.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_datatypes.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>

#define PI 3.14159265

bool explore;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

costmap_2d::Costmap2DROS* costmap_ros;
costmap_2d::Costmap2D costmap;

void alghoritmStateCallBack(const std_msgs::String& msg);
ros::Subscriber alghoritm_state_sub_;


class ChooseAccessibleBalls {

	ros::NodeHandle nh_;
	ros::Subscriber all_balls;
	ros::Publisher goal_pub_;
	ros::Publisher accesible_balls;
	bool first;
	tf::TransformListener tf_listener_;


public:
	MoveBaseClient ac;

	ChooseAccessibleBalls()  :
		ac("move_base", true){
		all_balls = nh_.subscribe < geometry_msgs::PoseArray > ("/allBallsXYZ", 10, &ChooseAccessibleBalls::allBallsCb, this);
		goal_pub_ = nh_.advertise < move_base_msgs::MoveBaseActionGoal > ("goal", 1);
		accesible_balls = nh_.advertise < geometry_msgs::PoseArray > ("accesible_balls", 1);

//		alghoritm_state_sub_ =  nh_.subscribe < std_msgs::String > ("alghoritm_state", 1, &ChooseAccessibleBalls::alghoritmStateCallBack, this);

		first = false;
		explore = true;
	}

	~ChooseAccessibleBalls() {
	}
	void allBallsCb(const geometry_msgs::PoseArrayConstPtr& all_balls_msg);

	void publishPose(float x, float y, geometry_msgs::Quaternion qMsg);
	void setAngle(double angle,  geometry_msgs::Quaternion& qMsg);
	void transFromCameraToOdomPosition(double x_cam_pose, double y_cam_pose, double z_cam_pose,
			double &x_odom_pose, double &y_odom_pose, double &z_odom_pose, tf::Transform& tfCP);
	void transFromOdomToMapPosition(double x_odom_pose, double y_odom_pose, double theta, double &x_map_pose, double &y_map_pose, tf::Quaternion& q);
	void getRobotPositionInOdom(double &x_odom_pose, double &y_odom_pose);
	bool canMove(float x, float y);
//	void setAngle(double angle,  geometry_msgs::Quaternion& qMsg);

	float getAngle(float x1, float y1, float x2, float y2);

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "chooseAccessibleBalls");
	ChooseAccessibleBalls chooseAccessibleBalls;

	tf::TransformListener tf_listener;
	costmap_ros = new costmap_2d::Costmap2DROS("local_costmap", tf_listener);

	ros::NodeHandle nh;
	alghoritm_state_sub_ =   nh.subscribe("alghoritm_state", 1, alghoritmStateCallBack);


	/*
	//wait for the action server to come up
	  while(!chooseAccessibleBalls.ac.waitForServer(ros::Duration(5.0))){
	    ROS_INFO("Waiting for the move_base action server to come up");
	  }
*/

	ros::spin();
	return 0;

}

void ChooseAccessibleBalls::allBallsCb(const geometry_msgs::PoseArrayConstPtr& all_balls_msg){
//	ROS_INFO("enter ChooseOneBall");

	/*

	if(explore == true){
		return;
	}

	*/
	ros::Time start = ros::Time::now();


	double robot_odom_x, robot_odom_y;
	getRobotPositionInOdom(robot_odom_x, robot_odom_y);
//	ROS_INFO("(robot_odom_x, robot_odom_y) (%f, %f)", robot_odom_x, robot_odom_y);

	std::vector<geometry_msgs::Pose> allBalls = all_balls_msg->poses;
//	ROS_INFO("number of balls = %d", (int)allBalls.size());

/*	if(first != false){
		return;
	}*/

	/*
	if(allBalls.size() != 1){
		return;
	}
	first = true;

*/


	geometry_msgs::PoseArray accesibleBallsMsg;
	accesibleBallsMsg.header.frame_id ="/odom";
	std::vector<geometry_msgs::Pose> poses;



	ros::Time past = all_balls_msg->header.stamp;
	tf::StampedTransform tfOC;								//	kamera w odom
	tf_listener_.waitForTransform("/odom", "/openni_rgb_optical_frame", past, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/openni_rgb_optical_frame", past,  tfOC);




	for (unsigned int i = 0; i < allBalls.size(); i++){

		double x = allBalls[i].position.x;
		double y = allBalls[i].position.y;
		double z = allBalls[i].position.z;

//		ROS_INFO("(x, y) (%f, %f)", x, y);



		double ball_odom_x, ball_odom_y, ball_odom_z;


		transFromCameraToOdomPosition(x, y, z, ball_odom_x, ball_odom_y, ball_odom_z, tfOC);


		if(ball_odom_z < 0.03 ||ball_odom_z > 0.05){
			continue;
		}

//		ROS_INFO("(x_odom, y_odom) (%f, %f)", x_odom, y_odom);

		float goal_odom_x, goal_odom_y;

		float dist = sqrt(pow(ball_odom_x - robot_odom_x, 2) + pow(ball_odom_y-robot_odom_y, 2) );
		ROS_INFO("dist = %f", dist);


		float dx = ball_odom_x - robot_odom_x;
		float dy = ball_odom_y - robot_odom_y;

		goal_odom_x = robot_odom_x + dx * (dist - .70) / dist;
		goal_odom_y = robot_odom_y + dy * (dist - .70) / dist;


		geometry_msgs::Pose pose;

		pose.position.x = ball_odom_x;
		pose.position.y = ball_odom_y;
		pose.position.z = ball_odom_z;

		pose.orientation = allBalls[i].orientation;

		poses.push_back(pose);





		bool canMove1 = canMove(goal_odom_x, goal_odom_y);

		if(canMove1==true){
	//		ROS_INFO("              can move to ball at (%f, %f)", goal_odom_x, goal_odom_y);

			geometry_msgs::Pose pose;

			pose.position.x = ball_odom_x;
			pose.position.y = ball_odom_y;
			pose.position.z = ball_odom_z;

			pose.orientation = allBalls[i].orientation;

			poses.push_back(pose);

		}
		else{
	//		ROS_INFO("              can not move to ball at (%f, %f)",goal_odom_x, goal_odom_y);
		}


	}

	ros::Time finish = ros::Time::now();
	ros::Duration duration = finish - start;
	double durSec = duration.toNSec()/1000000;
	double avgDur = durSec/allBalls.size();

	ROS_INFO("dur = %f [ms]", durSec);
	ROS_INFO("ball size = %d, average dur = %f [ms]", (int)allBalls.size(), avgDur);


//	if(poses.size() > 0 ){
		accesibleBallsMsg.poses = poses;
		accesible_balls.publish(accesibleBallsMsg);
//	}
}

void ChooseAccessibleBalls::publishPose(float x, float y, geometry_msgs::Quaternion qMsg){

	  move_base_msgs::MoveBaseGoal goal;

	  goal.target_pose.pose.position.x = x;
	  goal.target_pose.pose.position.y = y;

	  goal.target_pose.pose.orientation = qMsg;

	  goal.target_pose.header.stamp = ros::Time::now();
	  goal.target_pose.header.frame_id ="/map";


	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);

}

void ChooseAccessibleBalls::setAngle(double angle,  geometry_msgs::Quaternion& qMsg){

	btQuaternion q_result;
	q_result.setRPY(.0, .0, angle);
	tf::quaternionTFToMsg(q_result, qMsg);

}

/**
 * Metoda przeklada wspolrzedne pileczki w ukladzie kamery do ukladu odom
 */
void ChooseAccessibleBalls::transFromCameraToOdomPosition(double x_cam_pose, double y_cam_pose, double z_cam_pose,
		double &x_robot_pose, double &y_robot_pose, double &z_robot_pose, tf::Transform& tfOC){

//	tf::TransformListener tf_listener;
//	ros::Time now = ros::Time(0);
//	tf::StampedTransform tfOC;								//	kamera w odom
//	tf_listener.waitForTransform("/odom", "/openni_rgb_optical_frame", now, ros::Duration(1.0));
//	tf_listener.lookupTransform ("/odom", "/openni_rgb_optical_frame", now,  tfOC);

	tf::Transform tfCP;
	tfCP.setOrigin(tf::Vector3(x_cam_pose, y_cam_pose, z_cam_pose));			//	pileczka w kamerze


	tf::Transform tfOP = tfOC * tfCP;

	x_robot_pose = tfOP.getOrigin ().x();				//	wspolrzedne pileczki w ukladzie odom
	y_robot_pose = tfOP.getOrigin ().y();				//	wspolrzedne pileczki w ukladzie odom
	z_robot_pose = tfOP.getOrigin ().z();

}

bool ChooseAccessibleBalls::canMove(float x, float y){
	return true;

//	  ROS_INFO("enter canMove");
	  // Get a copy of the current costmap to test. (threadsafe)
	  costmap_2d::Costmap2D costmap;
//	  ROS_INFO("aaa");
	  if(costmap_ros != NULL)
		  costmap_ros->getCostmapCopy( costmap );

//	  ROS_INFO("bbb");
	  // Coordinate transform.
	  unsigned int cell_x, cell_y;
	  if( !costmap.worldToMap( x, y, cell_x, cell_y )){
	    return false;
	  }

	  double cost = double( costmap.getCost( cell_x, cell_y ));
	  if(cost <= 127){
		  return true;
	  }
	  else{
		  return false;
	  }
}

void ChooseAccessibleBalls::getRobotPositionInOdom(double &x_odom_pose, double &y_odom_pose){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/odom", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/base_link", now,  tfOR);

	x_odom_pose = tfOR.getOrigin ()[0];				//	wspolrzedne robota w ukladzie odom
	y_odom_pose = tfOR.getOrigin ()[1];				//	wspolrzedne robota w ukladzie odom

}

void ChooseAccessibleBalls::transFromOdomToMapPosition(double x_odom_pose, double y_odom_pose, double theta,
		double &x_map_pose, double &y_map_pose, tf::Quaternion& q){

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


float ChooseAccessibleBalls::getAngle(float x1, float y1, float x2, float y2){
	return atan2(y2-y1, x2-x1);
}


void alghoritmStateCallBack(const std_msgs::String& msg){
	ROS_INFO("alghoritmStateCallBack ");
	std::cout<<msg.data<<std::endl;
	if(msg.data.compare("SEARCH_BALLS") ==0 ){
		ROS_INFO("explore == true");
		explore = true;
	}else if(msg.data.compare("GO_TO_BALL") == 0 ){
		ROS_INFO("explore == false");
		explore = false;
	}
}



/**
 * Metoda sprawdza, czy pilka jest osiagalna. Wynzaczany jest okrag o srodku
 * w punkcie (x, y) a nastepnie sprawdzane jest, czy robot moze dojechac do tego okregu w odstepach
 *
 */

/*
bool ChooseAccessibleBalls::isBallAccessible(float x, float y){
	  // Get a copy of the current costmap to test. (threadsafe)
	  costmap_2d::Costmap2D costmap;

	  if(costmap_ros != NULL)
		  costmap_ros->getCostmapCopy( costmap );

	  // Coordinate transform.
	  unsigned int cell_x, cell_y;
	  if( !costmap.worldToMap( x, y, cell_x, cell_y )){
	    return false;
	  }

	  double cost = double( costmap.getCost( cell_x, cell_y ));
	  if(cost <= 127){
		  return true;
	  }
	  else{
		  return false;
	  }
}

*/




/*


double goal_map_x, goal_map_y, robot_map_x, robot_map_y;
tf::Quaternion q;
float angle = getAngle(robot_odom_x, robot_odom_y, ball_odom_x, ball_odom_y);

ROS_INFO("angle in odom = %f ", angle*180/PI);

transFromOdomToMapPosition(robot_odom_x, robot_odom_y, 0.0, robot_map_x, robot_map_y, q);
ROS_INFO("(robot_map_x, robot_map_y) (%f, %f)", robot_map_x, robot_map_y);


transFromOdomToMapPosition(goal_odom_x, goal_odom_y, angle, goal_map_x, goal_map_y, q);
ROS_INFO("(goal_odom_x, goal_odom_y) (%f, %f)", goal_odom_x, goal_odom_y);
ROS_INFO("(goal_map_x, goal_map_y) (%f, %f)", goal_map_x, goal_map_y);

//	ROS_INFO("angle in map = %f ", angle*180/PI);


double roll, pitch, yaw;
btMatrix3x3(q).getRPY(roll, pitch, yaw);
ROS_INFO("angle in map = %f ", yaw*180/PI);


geometry_msgs::Quaternion qMsg;
tf::quaternionTFToMsg(q, qMsg);

//		ROS_INFO(" (x, y, z, w) =  (%f, %f, %f, %f)", qMsg.x, qMsg.y, qMsg.z, qMsg.w );
publishPose(goal_map_x, goal_map_y, qMsg);

ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");


  */

