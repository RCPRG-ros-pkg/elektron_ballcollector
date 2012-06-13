#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>

#include <scheduler/SchedulerAction.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef enum
{
    EXPLORE = 0,
    GO_TO_BALL = 1,
    DEADLOCK = 2,
}State;


class Scheduler{

private:
	ros::NodeHandle nh_;
	ros::Subscriber selected_ball_sub_;
	ros::Subscriber deadlock_service_state_sub_;

	actionlib::SimpleActionClient<scheduler::SchedulerAction> deadlock_action_client_;
	actionlib::SimpleActionClient<scheduler::SchedulerAction> explore_action_client_;
	MoveBaseClient ac_;

	bool deadlock_;
	bool ball_visible_;

	State state_;




public:
	void selectedBallCb(const geometry_msgs::PointConstPtr& selectedBallPose);
	void deadlockServiceStateCb(const std_msgs::String& state);
	bool isDeadlock();
	bool isBallVisible();

	void sendDeadlockGoal();
//	void sendExploreGoal();
	void sendGoToBallGoal();

	void sendStartExploreGoal();
	void sendStopExploreGoal();

	bool isDeadlockServiceDone();
	bool isGoToBallServiceDone();

	void cancelAllGoals();


	State getState();
	void setSate(State state);



	Scheduler():
		deadlock_action_client_("get_out_of_deadlock", true),
		explore_action_client_("explore", true),
		ac_("move_base", true)
		{

		selected_ball_sub_ = nh_.subscribe < geometry_msgs::Point > ("/one_selected_ball", 1, &Scheduler::selectedBallCb, this);
		deadlock_service_state_sub_ = nh_.subscribe("/deadlock_service_state", 1, &Scheduler::deadlockServiceStateCb, this);
		deadlock_ = false;
		ball_visible_ = false;
		state_ = EXPLORE;

		ROS_INFO("Waiting for deadlock action server to start.");
		// wait for the action server to start
		deadlock_action_client_.waitForServer();
		ROS_INFO("Waiting for explore action server to start.");
		explore_action_client_.waitForServer();
		ROS_INFO("All action servers are ready");
	}



};

int main(int argc, char** argv) {
	ros::init(argc, argv, "scheduler");

	Scheduler scheduler;
	scheduler.sendStartExploreGoal();

	ros::Rate loop_rate(10);

	while (ros::ok()) {
			ros::spinOnce();
			loop_rate.sleep();

		//	std::cout<<"deadlock: "<<scheduler.isDeadlock()<<"  ball visible: "<<scheduler.isBallVisible()<<std::endl;


			//	eksploracja moze byc przerwana w dowolnym momencie tylko
			//	na podstawie wartosci zwracanych przez metody isDeadlock i isBallVisible
			//	metoda isDeadlock jest nadrzedna wzgledem isBallVisible, gdyz jesli robot jest zakleszczony,
			//	to nawet jesli widzi pileczke, to nie powinien do niej dojezdzac tylko wykonac obsluge zakleszczenia
			if(scheduler.getState() == EXPLORE){
				if (scheduler.isDeadlock()) {
					//	robot podczas eksploracji zakleszczyl sie
					//	przechodzimy do stanu obslugi zakleszczenia

					scheduler.cancelAllGoals();
					scheduler.sendStopExploreGoal();
					scheduler.sendDeadlockGoal();
					scheduler.setSate(DEADLOCK);

					ROS_INFO("EXPLORE ---> DEADLOCK");

				}
				else if(scheduler.isBallVisible()){
					//	robot podczas eksploracji zobaczyl pileczke, przechodzimy do stanu
					//	dojazdu do pileczki
					scheduler.cancelAllGoals();
					scheduler.sendGoToBallGoal();
					scheduler.setSate(GO_TO_BALL);
					ROS_INFO("EXPLORE ---> GO_TO_BALL");
				}

			}

			else if(scheduler.getState() == GO_TO_BALL){
			//	ROS_INFO("state = GO_TO_BALL");
				if (scheduler.isDeadlock()) {
					//	dojezdzal do pilki, ale sie zakleszczyl, np. probuje dojechac tam, gdzie nie moze
					//	przechodzimy do stany obslugi zakleszczenia

					scheduler.sendDeadlockGoal();
					scheduler.setSate(DEADLOCK);
					ROS_INFO("GO_TO_BALL ---> DEADLOCK");
					continue;
				}
				else if(!scheduler.isBallVisible()){
					//	robot dojezdza do pilki, ale przestal ja widziec,
					//	zaczyna eksplorowac otoczenie
					scheduler.sendStartExploreGoal();//sendExploreGoal();
					scheduler.setSate(EXPLORE);
					ROS_INFO("GO_TO_BALL ---> EXPLORE");
					continue;
				}

			}
			// 	ze stanu obslugi zakleszczenia wychodzmy tylko po zakonczeniu
			//	obslugi zakleszczenia. Przechodzimy wowczas do eksploracji
			else if(scheduler.getState() == DEADLOCK){
			//	ROS_INFO("state = DEADLOCK");
				if(scheduler.isDeadlockServiceDone()){
					//	zakonczono obsluge deadlocka
					//	zaczynamy exploracje
					scheduler.sendStartExploreGoal();
					scheduler.setSate(EXPLORE);
					ROS_INFO("DEADLOCK ---> EXPLORE");
				}
			}
	}


	return 0;
}


void Scheduler::selectedBallCb(const geometry_msgs::PointConstPtr& selectedBallPose){

//		ROS_INFO("selectedBallCb");
		if(selectedBallPose->x == 1000 && selectedBallPose->y == 1000 && selectedBallPose->z == 1000 ){
			ball_visible_ = false;
			return;
		}
		ball_visible_ = true;

}


void Scheduler::deadlockServiceStateCb(const std_msgs::String& state){
//	ROS_INFO("deadlockServiceStateCb");
	if(state.data == "DEADLOCK"){
		deadlock_ = true;
	}
	else if(state.data == "NOT_DEADLOCK"){
		deadlock_ = false;
	}
}

bool Scheduler::isDeadlock(){
	return deadlock_;

}
bool Scheduler::isBallVisible(){
	return ball_visible_;
}


void Scheduler::sendDeadlockGoal(){
	// send a goal to the action
//	ROS_INFO("send a goal to the deadlock action server");
	scheduler::SchedulerGoal goal;
	goal.value = 20;
	deadlock_action_client_.sendGoal(goal);
}

void Scheduler::sendStartExploreGoal(){
	// send a goal to the action
//	ROS_INFO("send a goal to the explore action server");
	scheduler::SchedulerGoal goal;
	goal.value = 1;
	explore_action_client_.sendGoal(goal);
}

void Scheduler::sendStopExploreGoal(){
	// send a goal to the action
//	ROS_INFO("send a goal to the explore action server");
	scheduler::SchedulerGoal goal;
	goal.value = 0;
	explore_action_client_.sendGoal(goal);
}


void Scheduler::sendGoToBallGoal(){

}

bool Scheduler::isDeadlockServiceDone(){

	bool is_done = deadlock_action_client_.getState().isDone();
	return is_done;
}

bool Scheduler::isGoToBallServiceDone(){
	return true;
}


State Scheduler::getState(){
	return state_;
}

void Scheduler::setSate(State state){
	state_ = state;
}

void Scheduler::cancelAllGoals(){
	ac_.cancelAllGoals ();
}

