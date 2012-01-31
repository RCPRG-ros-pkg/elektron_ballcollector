#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h>
#include "serialswitch.hpp"
#include <list>


// 	maksymalny rozmiar piłeczki - mieści się w kwadracie o boku 37px, czyli jej promień to ok 14px, czyli rozmiar 14^2*pi = ok. 615px
//	maksymalny obwód to 2*pi*r =


//	z odległości 100cm piłeczka ma rozmiar ok 360px
//	z odległości 70cm piłeczka ma rozmiar ok 655px

using namespace cv;

//SerialSwitch *sp;

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW_GREY[] = "Gray";

static const char WINDOW_ORG[] = "Orginal";

static const char WINDOW_COUNT[] = "Cpunture";


class PosAndId{
public:
	int x;
	int y;
	int id;
	PosAndId(int _id, int _x, int _y){
		x = _x;
		y= _y;
		id = _id;
	}
};


class ImageConverter {
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	image_transport::Subscriber image_depth_sub_;
	ros::Publisher vel_pub_;
	ros::Publisher state_pub_;

	int counter;

	int hooverOn;
	int robotRun;
	std::list<PosAndId*> ballList;


public:
	ImageConverter() :
			it_(nh_) {
		image_pub_ = it_.advertise("out", 1);
		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
				&ImageConverter::imageCb, this);

	//	image_depth_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
	//			&ImageConverter::imageCb, this);

		vel_pub_ = nh_.advertise<geometry_msgs::Twist> ("cmd_vel", 1);


		state_pub_ = nh_.advertise<std_msgs::Int16> ("state",1);



		counter = 0;
		namedWindow(WINDOW_GREY);
		namedWindow(WINDOW_ORG);
		namedWindow(WINDOW_COUNT);
		hooverOn = 0;
		robotRun = 1;

	}

	~ImageConverter() {
		destroyWindow(WINDOW_GREY);
		destroyWindow(WINDOW_ORG);
		destroyWindow(WINDOW_COUNT);
	}


	void onHoover(){
		if(hooverOn == 1)
			return;

		hooverOn = 1;
		switchHoover();

	}

	void offHoover(){
		if(hooverOn == 0)
			return;

		hooverOn = 0;
		//ros::Duration(1.0).sleep();
		switchHoover();

	}

	void switchHoover(){

		std_msgs::Int16 state;
		state.data = 1;
		state_pub_.publish(state);

	}

	void stopRobot(){
		if(robotRun == 0)
			return;
		robotRun =0;
		geometry_msgs::Twist vel;
		vel.angular.z = 0;
		vel.linear.x = 0;
		vel_pub_.publish(vel);
	}


	void imageCb(const sensor_msgs::ImageConstPtr& msg) {
//	  ROS_INFO("I heard Image");



		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8  );
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;

		}

		Mat gray, trash, orginal;

		Mat empty;

		cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

		threshold(gray, trash, 130, 255, THRESH_BINARY);



	//	adaptiveThreshold(gray, trash, 80, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 5, 2.0);

		Mat elem = (Mat_<int>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1);
		Point anchor = Point(-1, -1);
//		erode(trash, trash, elem, anchor, 1);
//		dilate(trash, gray, elem, anchor, 1);
//		dilate(trash, trash, elem, anchor, 1);
//		dilate(trash, trash, elem, anchor, 1);
//		dilate(gray, gray, elem, anchor, 1);


//		GaussianBlur( gray, gray, Size(9, 9), 2, 2 );

		Mat countMap = trash.clone();

		threshold(gray, empty, 255, 255, THRESH_BINARY);


		std::vector < std::vector<Point> > contours;
		findContours(countMap, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	//	ROS_INFO("There are %d contours", contours.size());

		Point2f center(0.f, 0.f);
		float radius =0.f;

		int count = 0;

		double area = 0.0;
		double area1= 0.0;
		double x=0.0;
		double y=0.0;
		double maxArea=0.0;
/*
		for (int i = 0; i < contours.size(); i++) {
			 area = contourArea(contours[i]);
			if (area < 100 ) {
					drawContours(gray, contours, i, 255);
			}
		}


*/



		for (int i = 0; i < contours.size(); i++) {
		//	drawContours(gray, contours, i, 255);
			 area = contourArea(contours[i]);
			 //1030
			if (area < 2030 && area > 100 ) {

				Moments mom = moments(contours[i]);

				double M11 = mom.m11 - (mom.m10 * mom.m01) / mom.m00;
				double M02 = mom.m02 - (mom.m01 * mom.m01) / mom.m00;
				double M20 = mom.m20 - (mom.m10 * mom.m10) / mom.m00;

				// for circle it should be ~0.0063
				double M7 = (M20 * M02 - M11 * M11)
						/ (mom.m00 * mom.m00 * mom.m00 * mom.m00);

				// circle
				if (M7 < 0.0065 && M7 > 0.0061){
					drawContours(cv_ptr->image, contours, i, CV_RGB(255, 0, 255));
				//	drawContours(gray, contours, i, CV_RGB(255, 0, 255));
					double r = sqrt(area/3.14);

					minEnclosingCircle(contours[i], center, radius );
					area1 = area;

					circle(empty, center, r, CV_RGB(255, 0, 255));


					if(area > maxArea){
						maxArea = area;
						x = center.x;
						y = center.y;
					}
					ROS_INFO("circle, M7 = %f , size = %f, odlegosc od kamery = %f, x=%f, y=%f",M7, area, 10.5/r, center.x, center.y);
					++count;

				//	ballList.push_back(new PosAndId(count, center.x, center.y));

				}
				// probably circle
				else if (M7 < 0.0085){
				//	ROS_INFO("probably circle, M7 = %f",M7);

				}
				// there is small chance that it's circular, but show them too
				else if (M7 < 0.011){
				//	ROS_INFO("no circle, M7 = %f",M7);
				}

			//	ROS_INFO("area0 = %f  area1 = %f", area0, area1);
			//	ROS_INFO("M7 = %f",M7);
			}
		}

/*



		if(count == 1 ){
			robotRun = 1;

					geometry_msgs::Twist vel;
					vel.angular.z = (320 - x)/500;
					vel.linear.x = (0.1 - (abs(320 - x) )/3200)*(1 - pow(y/460, 1) );

					vel_pub_.publish(vel);

					ROS_INFO("circle area1 =%f", maxArea);
					if(maxArea > 850 && abs(320 - x) < 50 ){
						ROS_INFO("start hoover");
						onHoover();
					//	vel.linear.x = 0;
						vel.linear.x = 0.2;
						vel_pub_.publish(vel);
						ros::Duration(1.0).sleep();
						vel.linear.x = 0;
						vel_pub_.publish(vel);
						ros::Duration(1.0).sleep();
						ROS_INFO("stop hoover");
					}
					else if(maxArea < 800 ){
						offHoover();
					}
		}
		else{
			offHoover();
			stopRobot();
		}

*/


	//	ROS_INFO("There are %d balls", count);

		imshow(WINDOW_COUNT, trash);

		imshow(WINDOW_GREY, empty);
		//  waitKey(3);

		imshow(WINDOW_ORG, cv_ptr->image);
		waitKey(3);

		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ic.switchHoover();
	ros::spin();
	return 0;
}






