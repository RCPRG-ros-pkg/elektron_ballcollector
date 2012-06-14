#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>


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
#include <std_msgs/Float32.h>

#include <nav_msgs/GridCells.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_datatypes.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>


#include <actionlib/server/simple_action_server.h>
#include <scheduler/SchedulerAction.h>


typedef enum
{
    GO_TO_BALL_WITH_NAV = 0,
    GO_FORWARD_INTRO = 1,
    GO_FORWARD_COLLECT = 2,
    GO_FORWARD_FINISH = 3,
    LOOKING_FOR_BALLS = 4,
    GO_TO_BALL = 5,
    IDLE = 6,


    STOP =7,
    FIRST_STEP_COLLECT = 8,
    SECOND_STEP_COLLECT = 9,
}State;




typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class GoToSelectedBall{

private:
	ros::NodeHandle nh_;
	ros::Subscriber selected_ball_sub_;
	ros::Subscriber go_forward_robot_state_sub_;
	ros::Subscriber deadlock_service_state_sub;

	geometry_msgs::Point current_pose_;
	ros::Publisher hoover_state_pub_;
	ros::Publisher alghoritm_state_pub_;

	ros::Publisher go_forward_robot_pub_;


	tf::TransformListener tf_listener_;

	actionlib::SimpleActionServer<scheduler::SchedulerAction> as_;
	std::string action_name_;

	State state_;

public:
	bool firstGoalSent;
	MoveBaseClient ac;
	bool isBallPoseSet;
	bool is_deadlock_service_run;

	int moveStraightState;			// 0 - nic, 1 - jedzie, 2 - dojechal
	bool moveStraightStateChange;


	void selectedBallCb(const geometry_msgs::PointConstPtr& selectedBallPose);
	void robotGoStraightStateCb(const std_msgs::Int16& state);
	void deadlockServiceStateCb(const std_msgs::String& state);

	scheduler::SchedulerFeedback feedback_;
	scheduler::SchedulerResult result_;

	GoToSelectedBall(std::string name) :
		as_(nh_, name, boost::bind(&GoToSelectedBall::executeCB, this, _1), false),
		action_name_(name),
		ac("move_base", true)
	{
		selected_ball_sub_ = nh_.subscribe < geometry_msgs::Point > ("/one_selected_ball", 1, &GoToSelectedBall::selectedBallCb, this);
		hoover_state_pub_ = nh_.advertise<std_msgs::Int16> ("hoover_state",1);
		go_forward_robot_pub_ = nh_.advertise< std_msgs::Float32>("/robot_go_straight",1);
		go_forward_robot_state_sub_ = nh_.subscribe("/robot_go_straight_state", 1, &GoToSelectedBall::robotGoStraightStateCb, this);
		alghoritm_state_pub_ = nh_.advertise<std_msgs::String> ("/alghoritm_state",1);
		deadlock_service_state_sub = nh_.subscribe("/deadlock_service_state", 1, &GoToSelectedBall::deadlockServiceStateCb, this);


		firstGoalSent = false;
		isBallPoseSet = false;
		moveStraightState = 0;
		moveStraightStateChange = false;
		is_deadlock_service_run = false;

		as_.start();
		state_ = STOP;

	}
	~GoToSelectedBall() {

	}


	geometry_msgs::Point getCurrentPose();
	float getAngle(float x1, float y1, float x2, float y2);
	void getRobotPositionInOdom(float &x_odom_pose, float &y_odom_pose);
	float getRobotAngleInOdom();
	void transFromOdomToMapPosition(float x_odom_pose, float y_odom_pose, float theta,
			float &x_map_pose, float &y_map_pose, tf::Quaternion& q);
	void publishPose(float dist_from_ball);
	void publishAngle();
	bool getFirstGoalSent(){return firstGoalSent;};
	float getDistanceFromSelectedBall();

	float getAngleDiff();

	void onHoover();
	void offHoover();
	void switchHoover();


	void goForward(float dist);
	void sleepWithSpin(int hectoseconds);

	void startExplore();
	void stopExplore();

	State getState(){return state_;};
	void setState(State state){state_ = state;};

	void executeCB(const scheduler::SchedulerGoalConstPtr &goal);

};



int main(int argc, char** argv) {
	ros::init(argc, argv, "goToSelectedBall");
	GoToSelectedBall goToSelectedBall(ros::this_node::getName());

	 while(!goToSelectedBall.ac.waitForServer(ros::Duration(5.0))){
	    ROS_INFO("Waiting for the move_base action server to come up");
	 }




	ros::Rate loop_rate(10);

	int no_ball_counter = 0;

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();


		if( goToSelectedBall.getState() == STOP ){
			ROS_INFO("STOP state");
			continue;
		}
		else if(goToSelectedBall.getState() == FIRST_STEP_COLLECT){
			//	scheduler zezwolil na jazde, ale node nie ma wspolrzednych pileczki
			if(goToSelectedBall.isBallPoseSet == false){
				ROS_INFO("FIRST_STEP_COLLECT - no ball visible");
				continue;
			}
			else{
			//	jest pileczka, sprwdzamy odleglosc
				if(goToSelectedBall.getDistanceFromSelectedBall() > 0.6){

					ROS_INFO("FIRST_STEP_COLLECT - go to ball");
					float angleDiffRobotGoal = goToSelectedBall.getAngleDiff()*180/(3.14);
					if(angleDiffRobotGoal > 2.5){
						goToSelectedBall.publishAngle();
						goToSelectedBall.ac.waitForResult();
						goToSelectedBall.goForward(0.1);
					}
					else{
					//	goToSelectedBall.publishPose(0.2);
						goToSelectedBall.goForward(0.1);
					}
				}
				else{
					ROS_INFO("FIRST_STEP_COLLECT - ball too close");
				}
			}
		}


		/*


		if(goToSelectedBall.isBallPoseSet == false){
				//	brak piłeczki
			++no_ball_counter;

			if(no_ball_counter > 20){
				goToSelectedBall.startExplore();
				goToSelectedBall.alg_state_ = LOOKING_FOR_BALLS;
				ROS_INFO("state = LOOKING_FOR_BALLS, wait for ball no_ball_counter = %d", no_ball_counter);
			}
			else{
				goToSelectedBall.ac.cancelAllGoals ();
				goToSelectedBall.alg_state_ = IDLE;
				ROS_INFO("state = IDLE, wait for ball no_ball_counter = %d", no_ball_counter);
			}

			continue;
		}
		else{

			if(goToSelectedBall.alg_state_ == IDLE || goToSelectedBall.alg_state_ == LOOKING_FOR_BALLS){
				goToSelectedBall.ac.cancelAllGoals ();
			}
			goToSelectedBall.stopExplore();

			goToSelectedBall.alg_state_ = GO_TO_BALL;

			no_ball_counter = 0;
				// jest piłeczka i nie jedzie prosto do piłeczki
			ROS_INFO("state = GO_TO_BALL, Go to ball using NAV");

			if(goToSelectedBall.getDistanceFromSelectedBall() > 0.6){
	//			goToSelectedBall.offHoover();

				float angleDiffRobotGoal = goToSelectedBall.getAngleDiff()*180/(3.14);
			//	ROS_INFO("angleDiffRobotGoal = %f", angleDiffRobotGoal);
				if(angleDiffRobotGoal > 2.5){
					goToSelectedBall.publishAngle();
					goToSelectedBall.ac.waitForResult();
					goToSelectedBall.goForward(0.1);
				}
				else{
				//	goToSelectedBall.publishPose(0.2);
					goToSelectedBall.goForward(0.1);
				}
			}

			else{
				float angleDiffRobotGoal = goToSelectedBall.getAngleDiff()*180/(3.14);
				if(angleDiffRobotGoal > 2.5){
					goToSelectedBall.publishAngle();
					goToSelectedBall.ac.waitForResult();
				}
				float dist = goToSelectedBall.getDistanceFromSelectedBall();
				goToSelectedBall.onHoover();
				goToSelectedBall.goForward(dist - 0.3);
				ros::Duration(4.0).sleep();
				goToSelectedBall.goForward(-(dist - 0.3));
				ros::Duration(4.0).sleep();
			}
		}


		*/

	}
}


float GoToSelectedBall::getDistanceFromSelectedBall(){

	float robot_odom_x, robot_odom_y, ball_odom_x, ball_odom_y;
	getRobotPositionInOdom(robot_odom_x, robot_odom_y);

	ball_odom_x = current_pose_.x;
	ball_odom_y = current_pose_.y;
//	ROS_INFO("robot (x, y) i odom = (%f, %f)", robot_odom_x, robot_odom_y);
//	ROS_INFO("ball (x, y) i odom = (%f, %f)", ball_odom_x, ball_odom_y);


	float dist = sqrt(pow(ball_odom_x - robot_odom_x, 2) + pow(ball_odom_y-robot_odom_y, 2) );

	return dist;
}


void GoToSelectedBall::selectedBallCb(const geometry_msgs::PointConstPtr& selectedBallPose){

//	ROS_INFO("selectedBallCb");
	if(selectedBallPose->x == 1000 && selectedBallPose->y == 1000 && selectedBallPose->z == 1000 ){
		isBallPoseSet = false;
//		ROS_INFO("ret 1");
		return;
	}

	current_pose_.x = selectedBallPose->x;
	current_pose_.y = selectedBallPose->y;
	isBallPoseSet = true;

//	ROS_INFO("enter selectedBallCb (x, y) = (%f, %f)", current_pose_.x, current_pose_.y);
	float angleDiffRobotGoal = getAngleDiff()*180/(3.14);
//	ROS_INFO("angleDiffRobotGoal = %f", angleDiffRobotGoal);
//	ROS_INFO("ret 2");

}


geometry_msgs::Point GoToSelectedBall::getCurrentPose(){
	return current_pose_;
}

void GoToSelectedBall::deadlockServiceStateCb(const std_msgs::String& state){
	if(state.data == "RUNNING"){
		is_deadlock_service_run = true;
	}
	else if(state.data == "NOT_RUNNING"){
		is_deadlock_service_run = false;
	}

}


void GoToSelectedBall::publishPose(float dist_from_ball){

	ROS_INFO("enter publishPose, distance = %f", dist_from_ball);

	if(isBallPoseSet== false){
		ROS_INFO("publishPose, wait for ball position, return");
		return;
	}

	float ball_odom_x = getCurrentPose().x;
	float ball_odom_y = getCurrentPose().y;


	float robot_odom_x, robot_odom_y;
	getRobotPositionInOdom(robot_odom_x, robot_odom_y);
//	ROS_INFO("robot (x, y) i odom = (%f, %f)", robot_odom_x, robot_odom_y);
//	ROS_INFO("ball (x, y) i odom = (%f, %f)", ball_odom_x, ball_odom_y);

	float dx = ball_odom_x - robot_odom_x;
	float dy = ball_odom_y - robot_odom_y;

	float dist = sqrt(pow(ball_odom_x - robot_odom_x, 2) + pow(ball_odom_y-robot_odom_y, 2) );

	ROS_INFO("dist = %f",dist);

	float goal_odom_x = robot_odom_x + dx * (dist - dist_from_ball) / dist;
	float goal_odom_y = robot_odom_y + dy * (dist - dist_from_ball) / dist;

	float angle = getAngle(robot_odom_x, robot_odom_y, ball_odom_x, ball_odom_y);

	tf::Quaternion q;
	float goal_map_x, goal_map_y;
	transFromOdomToMapPosition(goal_odom_x, goal_odom_y, angle, goal_map_x, goal_map_y, q);


//	float roll, pitch, yaw;
//	btMatrix3x3(q).getRPY(roll, pitch, yaw);
//	ROS_INFO("angle in map = %f ", yaw*180/PI);

	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(q, qMsg);


	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.pose.position.x = goal_map_x;
	goal.target_pose.pose.position.y = goal_map_y;
	goal.target_pose.pose.position.z = 0;


	goal.target_pose.pose.orientation = qMsg;

	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id ="/map";


	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	firstGoalSent = true;

/*

	 ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, the base moved 1 meter forward");
	else
		ROS_INFO("The base failed to move forward 1 meter for some reason");
*/
//	isBallPoseSet = false;


}


void GoToSelectedBall::publishAngle(){

	ROS_INFO("enter publishAngle");

	if(isBallPoseSet== false){
		ROS_INFO("publishAngle, wait for ball position, return");
		return;
	}


	float ball_odom_x = getCurrentPose().x;
	float ball_odom_y = getCurrentPose().y;

	float robot_odom_x, robot_odom_y;
	getRobotPositionInOdom(robot_odom_x, robot_odom_y);

	float goal_odom_x = robot_odom_x;// + dx * (dist - dist_from_ball) / dist;
	float goal_odom_y = robot_odom_y;// + dy * (dist - dist_from_ball) / dist;

	float angle = getAngle(robot_odom_x, robot_odom_y, ball_odom_x, ball_odom_y);

	tf::Quaternion q;
	float goal_map_x, goal_map_y;
	transFromOdomToMapPosition(goal_odom_x, goal_odom_y, angle, goal_map_x, goal_map_y, q);

	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(q, qMsg);


	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.pose.position.x = goal_map_x;
	goal.target_pose.pose.position.y = goal_map_y;
	goal.target_pose.pose.position.z = 0;


	goal.target_pose.pose.orientation = qMsg;

	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id ="/map";


	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	firstGoalSent = true;
//	isBallPoseSet = false;

	ROS_INFO("return publishAngle");
}





float GoToSelectedBall::getAngle(float x1, float y1, float x2, float y2){
	return atan2(y2-y1, x2-x1);
}


void GoToSelectedBall::getRobotPositionInOdom(float &x_odom_pose, float &y_odom_pose){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/odom", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/base_link", now,  tfOR);

	x_odom_pose = tfOR.getOrigin ()[0];				//	wspolrzedne robota w ukladzie odom
	y_odom_pose = tfOR.getOrigin ()[1];				//	wspolrzedne robota w ukladzie odom

}


void GoToSelectedBall::transFromOdomToMapPosition(float x_odom_pose, float y_odom_pose, float theta,
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



void GoToSelectedBall::onHoover() {
	std_msgs::Int16 state;
	state.data = 0;
	hoover_state_pub_.publish(state);

}

void GoToSelectedBall::offHoover() {
	std_msgs::Int16 state;
	state.data = 1;
	hoover_state_pub_.publish(state);

}

void GoToSelectedBall::switchHoover() {

	std_msgs::Int16 state;
	state.data = 2;
	hoover_state_pub_.publish(state);

}


float GoToSelectedBall::getAngleDiff(){

	float robot_odom_x, robot_odom_y;
	getRobotPositionInOdom(robot_odom_x, robot_odom_y);

	float goalAngle = getAngle(robot_odom_x, robot_odom_y, getCurrentPose().x, getCurrentPose().y);
	float robotAngle = getRobotAngleInOdom();
//	std::cout<<"goalAngle = "<<goalAngle<<"robotAngle = "<<robotAngle<<std::endl;
	return fabs(goalAngle-robotAngle);

}

float GoToSelectedBall::getRobotAngleInOdom(){

	ros::Time now = ros::Time(0);
	tf::StampedTransform tfOR;								//	robot w odom
	tf_listener_.waitForTransform("/odom", "/base_link", now, ros::Duration(1.0));
	tf_listener_.lookupTransform ("/odom", "/base_link", now,  tfOR);

//	x_odom_pose = tfOR.getOrigin ()[0];				//	wspolrzedne robota w ukladzie odom
//	y_odom_pose = tfOR.getOrigin ()[1];				//	wspolrzedne robota w ukladzie odom

	return tf::getYaw(tfOR.getRotation());

//	float roll, pitch, yaw;
//	btMatrix3x3( tfOR.getRotation() ).getRPY(roll, pitch, yaw);

//	return yaw;
}

void GoToSelectedBall::goForward(float dist){
	std_msgs::Float32 go;
	go.data = dist;
	go_forward_robot_pub_.publish(go);
}


void GoToSelectedBall::sleepWithSpin(int hectoseconds){

	ROS_INFO("enter sleepWithSpin, hectoseconds = %d", hectoseconds);
	for(int i=0; i<hectoseconds; ++i){
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	}
	ROS_INFO("leave sleepWithSpin");

}

void GoToSelectedBall::robotGoStraightStateCb(const std_msgs::Int16& state){
//	ROS_INFO("enter robotGoStraightStateCb state = %d", state.data);

	if(moveStraightState != state.data){
		moveStraightStateChange = true;
	}
	else{
		moveStraightStateChange = false;
	}
	moveStraightState = state.data;
}

void GoToSelectedBall::startExplore(){

	std_msgs::String state;
	state.data = "SEARCH_BALLS";
	alghoritm_state_pub_.publish(state);
}
void GoToSelectedBall::stopExplore(){

	std_msgs::String state;
	state.data = "GO_TO_BALL";
	alghoritm_state_pub_.publish(state);
}

void GoToSelectedBall::executeCB(const scheduler::SchedulerGoalConstPtr &goal){
	ROS_INFO("enter executeCB, goal = %i", goal->value);

	if(goal->value == 0){
		state_ = STOP;
	}
	else if(goal->value == 1){
		state_ = FIRST_STEP_COLLECT;
	}
	else if(goal->value == 2){
		// TODO: sprawdza, czy jest ustawiona pozycja pileczki, albo przesylac ja razem z goalem
		state_ = SECOND_STEP_COLLECT;

		ROS_INFO("enter SECOND_STEP_COLLECT");
		float angleDiffRobotGoal = getAngleDiff()*180/(3.14);
		if(angleDiffRobotGoal > 2.5){
			publishAngle();
			ac.waitForResult();
		}
		float dist = getDistanceFromSelectedBall();
		onHoover();
		goForward(dist - 0.3);
		ros::Duration(4.0).sleep();
		goForward(-(dist - 0.3));
		ros::Duration(4.0).sleep();
		ROS_INFO("leave SECOND_STEP_COLLECT");
	}


	as_.publishFeedback(feedback_);
	result_.value = feedback_.value;

	as_.setSucceeded(result_);
	ROS_INFO("leave executeCB");
}



/*
			if(goToSelectedBall.getDistanceFromSelectedBall() > 1.0){
				goToSelectedBall.offHoover();
				goToSelectedBall.publishPose(goToSelectedBall.getCurrentPose().x, goToSelectedBall.getCurrentPose().y, 0.9);
			}
			else if(goToSelectedBall.getDistanceFromSelectedBall() > 0.9){
				goToSelectedBall.offHoover();
				goToSelectedBall.publishPose(goToSelectedBall.getCurrentPose().x, goToSelectedBall.getCurrentPose().y, 0.8);
			}
			else if(goToSelectedBall.getDistanceFromSelectedBall() > 0.8){
				goToSelectedBall.offHoover();
				goToSelectedBall.publishPose(goToSelectedBall.getCurrentPose().x, goToSelectedBall.getCurrentPose().y, 0.7);
			}
			else if(goToSelectedBall.getDistanceFromSelectedBall() > 0.7){
				goToSelectedBall.offHoover();
				goToSelectedBall.publishPose(goToSelectedBall.getCurrentPose().x, goToSelectedBall.getCurrentPose().y, 0.6);
			}
			*/


			/*

			if(goToSelectedBall.getDistanceFromSelectedBall() > 1.0){
				goToSelectedBall.offHoover();
				goToSelectedBall.publishPose(goToSelectedBall.getCurrentPose().x, goToSelectedBall.getCurrentPose().y, 0.8);
			}
			else if(goToSelectedBall.getDistanceFromSelectedBall() > 0.8){
				goToSelectedBall.offHoover();
				goToSelectedBall.publishPose(goToSelectedBall.getCurrentPose().x, goToSelectedBall.getCurrentPose().y, 0.6);
			}

			else{
				goToSelectedBall.onHoover();
				goToSelectedBall.publishPose(goToSelectedBall.getCurrentPose().x, goToSelectedBall.getCurrentPose().y, 0.1);
				goToSelectedBall.ac.waitForResult();

		//		goToSelectedBall.publishPose(goToSelectedBall.getCurrentPose().x, goToSelectedBall.getCurrentPose().y, 0.0);
		//		goToSelectedBall.ac.waitForResult();
				goToSelectedBall.offHoover();
			}

			*/




/*

		if(goToSelectedBall.moveStraightState == 1){
				// jedzie do piłeczki, nic nie robimy
			ROS_INFO("wait, robot go straight");
			continue;
		}

		if(goToSelectedBall.moveStraightState == 2){
			ROS_INFO("Robot reached goal ");
			//	dojechal, sprawdzamy jaki stan byl poprzednio i wyznaczamy kolejny i sterowanie

			if(goToSelectedBall.alg_state_ == GO_FORWARD_INTRO){
				//	robot przejechal pierwszy odcinek do pilki, teraz czas na drugi
				ROS_INFO("First part OK ");
				goToSelectedBall.alg_state_ = GO_FORWARD_COLLECT;

				goToSelectedBall.onHoover();
				goToSelectedBall.goForward(0.2);

			}
			else if(goToSelectedBall.alg_state_ == GO_FORWARD_COLLECT){
				//	robot przejechal pierwszy odcinek do pilki, teraz czas na drugi
				ROS_INFO("Second part OK ");
				goToSelectedBall.alg_state_ = GO_FORWARD_FINISH;

				goToSelectedBall.offHoover();
				goToSelectedBall.goForward(0.2);

			}

			else if(goToSelectedBall.alg_state_ == GO_FORWARD_FINISH){
			//	robot przejechal ostatni odcinek, pilka powinna byc juz wciagnieta
				ROS_INFO("Third part OK");

				// zaczynamy szukac pilek
				goToSelectedBall.alg_state_ = LOOKING_FOR_BALLS;
			}
		}
		else if(goToSelectedBall.isBallPoseSet== false){
				//	brak piłeczki
			ROS_INFO("wait for ball");
			continue;
		}
		else{
				// jest piłeczka i nie jedzie prosto do piłeczki
			ROS_INFO("Go to ball using NAV");

			if(goToSelectedBall.getDistanceFromSelectedBall() > 0.7){
				goToSelectedBall.offHoover();

				float angleDiffRobotGoal = goToSelectedBall.getAngleDiff()*180/(3.14);
			//	ROS_INFO("angleDiffRobotGoal = %f", angleDiffRobotGoal);
				if(angleDiffRobotGoal > 2.5){
					goToSelectedBall.publishAngle();
				}
				else{
					goToSelectedBall.publishPose(0.2);
				}
			}

			else{
				float angleDiffRobotGoal = goToSelectedBall.getAngleDiff()*180/(3.14);
				if(angleDiffRobotGoal > 2.5){
					goToSelectedBall.publishAngle();
					goToSelectedBall.ac.waitForResult();
				}

				goToSelectedBall.ac.cancelAllGoals ();


				ROS_INFO("go forward");
				goToSelectedBall.goForward(0.2);
				goToSelectedBall.alg_state_ = GO_FORWARD_INTRO;



			}
		}

		*/




