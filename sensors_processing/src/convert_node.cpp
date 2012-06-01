//#define _ON_PC_ 1

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <inttypes.h>
#include <cv.h>
#include <math.h>
#include <list>
#include <ecl/ipc.hpp>

#include "../../simple_apps/src/serialswitch.hpp"

#include <dynamic_reconfigure/server.h>
#include <sensors_processing/TutorialsConfig.h>

#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int16MultiArray.h>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>


#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


using namespace cv;




using ecl::Semaphore;

typedef struct myCircle {
	uint16_t x;
	uint16_t y;
	uint16_t r;
	uint16_t depth;
} CIRCLE;

std::vector<CIRCLE> circleList;

double val1 = 6.0;
double val2 = 16.0;
int morphIterations = 1;
int ringWeight = 9;
int robotRun = 1;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


void callback(sensors_processing::TutorialsConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %f %f", config.double_param1,
			config.double_param2);

	val1 = config.double_param1;
	val2 = config.double_param2;
	morphIterations = config.morphIterations;
	ringWeight = config.ringWeight;
}

int getDistance(CIRCLE circle);
int getClosestId(std::vector<CIRCLE> circles );
void drawAllCrircles(std::vector<CIRCLE> circles, Mat mat );


void publishAlgState(string state);
void publishAllBalls();

/**
 * Metoda przeklada wspolrzedne 
 * 
 **/
void transformFromImageToOdom(double x_image, double y_image, double &x_odom, double &y_odom);


namespace enc = sensor_msgs::image_encodings;

bool fristLoop = true;
Mat firstImage;
Mat out_image;

#ifdef _ON_PC_

static const char WINDOW_GREY[] = "Gray";
static const char WINDOW_ORG[] = "Orginal";
static const char WINDOW_COUNT[] = "Cpunture";
static const char WINDOW_OUT[] ="Out";

static const char WINDOW_COLOR_1[] ="Image";

#endif

class ImageConverter {
	ros::NodeHandle nh_;

	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	image_transport::Subscriber image_depth_sub_;
	ros::Publisher vel_pub_;
	ros::Publisher state_pub_;
	ros::Publisher alghoritm_state_pub_;				// 	0- explore, 1-to ball
	ros::Publisher balls_pub_;							//	publikuje wszsytkie pileczki

	ros::Subscriber point_cloud_sub_;


	int counter;

	int hooverOn;
	bool explore;

	int hysteresis;

public:
	MoveBaseClient ac;
	void stopExplore();
	void startExplore();

	ImageConverter() :
		it_(nh_), ac("move_base", true) {

		image_depth_sub_ = it_.subscribe("depth_reg", 1, &ImageConverter::depthImageCb, this);
		image_sub_ = it_.subscribe("image_color", 1, &ImageConverter::imageCb, this);

		vel_pub_ = nh_.advertise < geometry_msgs::Twist > ("cmd_vel", 1);
		state_pub_ = nh_.advertise < std_msgs::Int16 > ("state", 1);
		image_pub_ = it_.advertise("out", 1);

		alghoritm_state_pub_ =   nh_.advertise < std_msgs::String > ("alghoritm_state", 1);
		balls_pub_ = nh_.advertise < geometry_msgs::PoseArray > ("allBalls", 1);

	//	cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//	point_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 10, &ImageConverter::PTScallback, this);
		point_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2> ("cloud_in", 10, &ImageConverter::PTScallback, this);



		counter = 0;
		hooverOn = 0;

		hysteresis = 0;

		Semaphore semaphore("test_sem");
		semaphore.unlock();
		explore = false;

		 val1 = 6.0;
		 val2 = 16.0;
		 morphIterations = 1;
		 ringWeight = 9;


#ifdef _ON_PC_

		namedWindow(WINDOW_GREY);
		namedWindow(WINDOW_ORG);
		namedWindow(WINDOW_COUNT);
		namedWindow(WINDOW_OUT);
		namedWindow(WINDOW_COLOR_1);
#endif

	}

	~ImageConverter() {

#ifdef _ON_PC_

		destroyWindow(WINDOW_GREY);
		destroyWindow(WINDOW_ORG);
		destroyWindow(WINDOW_COUNT);
		destroyWindow(WINDOW_OUT);
		destroyWindow(WINDOW_COLOR_1);

#endif
	}


	/**
	 * Callback obrazka kolorowego. Teraz nie jest wykorzystywany
	 * w algorytmie. sluzy tylko do informacji.
	 */
	void imageCb(const sensor_msgs::ImageConstPtr& msg) {
//		ROS_INFO("I heard color image");

		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;

		}

		Mat gray;
//		int count = 0;
//		double area = 0.0;
//		double area1 = 0.0;
//		double x = 0.0;
//		double y = 0.0;
//		double maxArea = 0.0;

		cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

		Semaphore semaphore("test_sem");
		semaphore.lock();
		
		
		//              ROS_INFO("imageCb semaphore in");

//		int circleIndex = -1;

		int minDistIndex = -1;
//		int minDist = 10000;


		drawAllCrircles(circleList, gray);

		//	przechodzimy po wszystkich okręgach
		//	i szukamy okręgu leżącego najbliżej
		
		minDistIndex = getClosestId(circleList);
		

		//	jeśli istnieje jakiś okrąg
		if (minDistIndex != -1) {


/*
			++hysteresis;

			ROS_INFO("          hysteresis = %d",hysteresis );
			if(hysteresis < 1){
				return;
			}
			hysteresis = 1;
*/

//			publishAlgState("GO_TO_BALL");
//			stopExplore();

			robotRun = 1;
//			ROS_INFO("minDistIndex=%d ", minDistIndex);

			int circleX = circleList[minDistIndex].x;
			int circleY = circleList[minDistIndex].y;

			// rysujemy okrąg, do którego jedzie robot
			circle(gray, Point(circleX, circleY) , 20, 0);

			//	wyznaczamy sterowanie silników

			geometry_msgs::Twist vel;
			vel.angular.z = (320.0 - (float) circleX) / 500.0;
			vel.linear.x = (0.1 - (abs(320 - circleX)) / 3200.0) * (1 - pow(circleY / 460.0, 1));


			//vel_pub_.publish(vel);
			//ROS_INFO("circle area1 =%f (x,y)=(%d,%d) vel.linear.x = %f, vel.angular.z = %f",
			//		maxArea, circleX, circleY, vel.linear.x, vel.angular.z);
			
			//	jesli robot podjechal wystarczajaco blisko to zasysa pilke
			if (circleY > 430 && circleX > 315 && circleX < 325) {
//				suckBall();
			}

		} else {

			/*
			--hysteresis;

			if(hysteresis > -1){
				return;
			}
			hysteresis = -1;

			startExplore();
			offHoover();

			*/
		//	stopRobot();
		}




		semaphore.unlock();



		cv_bridge::CvImage out_msg;
		out_msg.header = cv_ptr->header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
		out_msg.image = gray; // Your cv::Mat




		image_pub_.publish(out_msg.toImageMsg());

		//      image_pub_.publish(gray);

#ifdef _ON_PC_
		imshow(WINDOW_COLOR_1, gray);

		waitKey(3);
#endif
		//              ROS_INFO("imageCb semaphore out");

	}

	void suckBall(){
		ROS_INFO("start hoover");
		onHoover();
		//      vel.linear.x = 0;
		geometry_msgs::Twist vel;
		vel.angular.z = 0;
		vel.linear.x = 0.2;
		vel_pub_.publish(vel);
		ros::Duration(1.0).sleep();
		vel.linear.x = 0;
		vel_pub_.publish(vel);
		ros::Duration(1.0).sleep();
		offHoover();
		ros::Duration(3.0).sleep();
		ROS_INFO("stop hoover");
	}


	void depthImageCb(const sensor_msgs::ImageConstPtr& msg) {
//		ROS_INFO("I heard depth image");

		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_16UC1);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;

		}

		Mat gray, trash, orginal;

		cv_ptr->image.convertTo(gray, CV_8UC1, 1. / 16);
		cv_ptr->image.convertTo(orginal, CV_16UC1, 1);



		cv_ptr->image.convertTo(gray, CV_8UC1, 1. / 16);

		if (fristLoop) {
			out_image = gray.clone();

			for (int x = 0; x < 640; ++x) {
				for (int y = 0; y < 480; ++y) {
					out_image.at < uint8_t > (y, x) = 0;
				}
			}

			firstImage = orginal.clone();
			fristLoop = false;
		}

		for (int x = 0; x < 640; ++x) {
			for (int y = 0; y < 480; ++y) {
				out_image.at < uint8_t > (y, x) *= 0.5;
			}
		}
		for (int x = 0; x < 640; ++x) {
			for (int y = 0; y < 200; ++y) {
				gray.at < uint8_t > (y, x) = 0;
			}
		}

		threshold(gray, trash, 190, 255, THRESH_BINARY);

		uint16_t pixVal = orginal.at < uint16_t > (200, 320);

//              ROS_INFO("(320, 200)=  %d ", (int)pixVal);

		//orginal.at<uint16_t>(200, 320)= 60000;

		pixVal = orginal.at < uint16_t > (400, 320);

		//      ROS_INFO("(320, 400) =  %d ", (int)pixVal);

		std::vector < std::vector<Point> > contours;
		//      findContours(countMap, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		//      GaussianBlur(gray, gray, Size(9, 9), 2, 2);


		Canny(gray, gray, val1, val2, 3);

		Mat elem = (Mat_<int>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1);
		Point anchor = Point(-1, -1);

		dilate(gray, gray, elem, anchor, 1);

//              cv.MorphologyEx(image, image5, temp, element, cv.CV_MOP_CLOSE, 2);

		Mat countMap = gray.clone();

		morphologyEx(countMap, countMap, MORPH_CLOSE, elem, anchor,
				morphIterations);

//      InputArray src, OutputArray dst, int op, InputArray element, Point anchor=Point(-1,-1), int iterations=1,
//      int borderType=BORDER_CONSTANT, const Scalar& borderValue=morphologyDefaultBorderValue()

//      ROS_INFO("There are %d contours", contours.size());

		Mat empty = gray.clone();
		threshold(gray, empty, 255, 255, THRESH_BINARY);

		findContours(countMap, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

		//      ROS_INFO("There are %d contours\n", contours.size());

		Semaphore semaphore("test_sem");
		semaphore.lock();



		circleList.clear();
		for (unsigned int i = 0; i < contours.size(); i++) {
			//       drawContours( gray, contours[i], color, color, -1, CV_FILLED, 8 );


//			int area = contourArea(contours[i]);

			Point pt1 = Point(639, 479);
			Point pt2 = Point(0, 0);

			for (unsigned int j = 0; j < contours[i].size(); ++j) {
				pt1.x = min(pt1.x, contours[i][j].x);
				pt1.y = min(pt1.y, contours[i][j].y);

				pt2.x = max(pt2.x, contours[i][j].x);
				pt2.y = max(pt2.y, contours[i][j].y);
			}

			int a = (pt2.x - pt1.x);

			if (a < 10 || a > 40) {
				continue;
			}
			int center_y = (pt1.y + pt2.y) / 2;
			if (center_y < 150) {
				continue;
			}
			//               ROS_INFO("area %d = %d", i, area);
			pt2.y = pt1.y + a;

			// rectangle(empty, pt1, pt2, 255);
			//       rectangle(gray, pt1, pt2, 255);

//			int xOffset = pt1.x;
//			int yOffset = pt2.y;
//			int width = pt2.x - pt1.x;

			//for(int _x = pt1.x; _x)

			if (pt2.y > 479 || pt2.x > 639) {
				continue;
			}

			Range rowRange = Range(pt1.y, pt2.y);
			Range colRange = Range(pt1.x, pt2.x);

			//              ROS_INFO("draw  %d rowRange (%d,%d) - colRange (%d,%d)",i, rowRange.start, rowRange.end, colRange.start, colRange.end);
			//              ROS_INFO("mat  %d rows %d - cols %d",i, gray.rows, gray.cols);

			Mat currMat = Mat(gray, rowRange, colRange).clone();

			int cols = currMat.cols;
			int rows = currMat.rows;

//			int numberOfPixels = cols * rows;
			int x0 = cols / 2;
			int y0 = rows / 2;
			int r = cols / 2;

			int numberOfWhitePointInCircle = 0;
			int numberOfWhitePointOutCircle = 0;

	//		ROS_INFO("aaa");
	//		ROS_INFO("cols = %d, rows = %d", cols, rows);
			for (int x = 0; x < cols; ++x) {
				for (int y = 0; y < rows; ++y) {
		//			ROS_INFO("(x, y) = (%d, %d)", x,y);
					if (currMat.at < uint16_t > (y, x) == 255) {
						//        białe

						float xSq = pow(x - x0, 2);
						float ySq = pow(y - y0, 2);

						if (xSq + ySq <= pow(r + ringWeight, 2)
								&& xSq + ySq >= pow(r - ringWeight, 2)) {
							++numberOfWhitePointInCircle;
							//         punkt biały należy do okręgu - plus
						} else {
							//  punkt biały nie należy do okręgu - pinus
							++numberOfWhitePointOutCircle;
						}
					} else {
						// pomijamy dalszą analizę punktów czarnych
					}
				}
			}
	//		ROS_INFO("bbb");

//			int ob = 3 * r;


			if (numberOfWhitePointInCircle > numberOfWhitePointOutCircle) {

				CIRCLE circle;
				circle.r = r;
				circle.x = pt1.x + circle.r;
				circle.y = pt1.y + circle.r;

		//		ROS_INFO("x = %d, y = %d", circle.x, circle.y);

				if (circle.r < 10 || circle.r > 20) {
					continue;
				}
	//			ROS_INFO("(x, y)  = (%d, %d)", )
				circle.depth = orginal.at < uint16_t > (circle.y, circle.x);

				if(circle.y < 2 || circle.y > 476 || circle.x < 2 || circle.x > 636){
					continue;
				}
			//	ROS_INFO("ccc");

				int depth = 0;
				int number_of_elem = 0;
				for(int k = 0; k < 5; k ++){
					for(int m = 0; m < 5; m ++){
						int curr_dep_ = orginal.at < uint16_t > (circle.y -2+m, circle.x-2+k);
						if(curr_dep_ == 0){

						}
						else if(curr_dep_ > 2000){

						}
						else{
							depth += curr_dep_;
							++number_of_elem;
						//	ROS_INFO(" curr depth = %d  ", curr_dep_);
						}

					}
				}
			//	ROS_INFO("ddd");

			//	ROS_INFO("number_of_elem = %d", number_of_elem);

				if(number_of_elem == 0){
					continue;
				}

				depth /= number_of_elem;
				circle.depth = depth;


//				float lowR = -((float)circle.depth/87.0)+24.1;

				float lowR = -((float)circle.depth/87.0)+22.1;
				float hiR = -((float)circle.depth/87.0)+28.1;

//				ROS_INFO(" r = %d,   depth = %d,  lowR = %f,  hiR = %f", circle.r, circle.depth, lowR, hiR );
			//	int dep =  orginal.at < uint16_t > (351, 526);
			//	ROS_INFO("dep = %d", dep);





				if( (float)circle.r < lowR ||  (float)circle.r > hiR ){
					continue;
				}

				circleList.push_back(circle);


			}



/*

			drawContours(empty, contours, i, 255);

			// rectangle(gray, pt1, pt2, 255, CV_FILLED);
			// rectangle(out_image, pt1, pt2, 255, CV_FILLED);

			//               Point center((pt1.x + pt2.x)/2, (pt1.y + pt2.y)/2);
			//               circle(out_image,  center, a/2, 255, CV_FILLED);

			for (int x = pt1.x; x < pt2.x; ++x) {
				for (int y = pt1.y; y < pt2.y; ++y) {

					out_image.at < uint8_t > (y, x) =
							out_image.at < uint8_t > (y, x) < 200 ?
									out_image.at < uint8_t > (y, x) + 50 :
									out_image.at < uint8_t > (y, x);
				}
			}

			*/

		}

		/*

		threshold(out_image, trash, 75, 255, THRESH_BINARY);

		Mat out_image_copy = out_image.clone();

		findContours(out_image_copy, contours, CV_RETR_LIST,
				CV_CHAIN_APPROX_NONE);

//                Semaphore semaphore("test_sem");
//                semaphore.lock();
		//      ROS_INFO("imageDepthCb semaphore in");

	//	  circleList.clear();

		for (unsigned int i = 0; i < contours.size(); i++) {

			Point pt1 = Point(639, 479);
			Point pt2 = Point(0, 0);

			for (unsigned int j = 0; j < contours[i].size(); ++j) {
				pt1.x = min(pt1.x, contours[i][j].x);
				pt1.y = min(pt1.y, contours[i][j].y);

				pt2.x = max(pt2.x, contours[i][j].x);
				pt2.y = max(pt2.y, contours[i][j].y);
			}

			int a = (pt2.x - pt1.x);

		}


		*/

	//	ROS_INFO("circleList.size = %d", circleList.size());
		publishAllBalls();
		
//      ROS_INFO("imageDepthCb semaphore out");
		semaphore.unlock();

#ifdef _ON_PC_

		imshow(WINDOW_COUNT, trash);

		imshow(WINDOW_GREY, gray);
		//  waitKey(3);

		imshow(WINDOW_ORG, countMap);

		imshow(WINDOW_OUT, out_image);

		waitKey(3);

#endif

	//	      image_pub_.publish(cv_ptr->toImageMsg());
	}
	void stopRobot() {
		if (robotRun == 0)
			return;
		robotRun = 0;
		geometry_msgs::Twist vel;
		vel.angular.z = 0;
		vel.linear.x = 0;
		vel_pub_.publish(vel);
	}

	void onHoover() {
		if (hooverOn == 1)
			return;

		hooverOn = 1;
		switchHoover();

	}

	void offHoover() {
		if (hooverOn == 0)
			return;

		hooverOn = 0;
		//ros::Duration(1.0).sleep();
		switchHoover();

	}

	void switchHoover() {

		std_msgs::Int16 state;
		state.data = 1;
		state_pub_.publish(state);

	}

	void publishAlgState(string state){
		std_msgs::String stateMsg;
		stateMsg.data = state;
		alghoritm_state_pub_.publish(stateMsg);
	}
	
	void publishAllBalls(){
		
		geometry_msgs::PoseArray allBalls;
		allBalls.header.stamp = ros::Time(0);	//	now
		
		allBalls.header.frame_id ="/map";

	//	reprojectImageTo3D(InputArray disparity, OutputArray _3dImage, InputArray Q)
		

		std::vector<geometry_msgs::Pose> poses;

		for (unsigned int i = 0; i < circleList.size(); ++i) {

			geometry_msgs::Pose pose;
			pose.position.x = circleList[i].x;
			pose.position.y = circleList[i].y;
			pose.position.z = circleList[i].depth;
			poses.push_back(pose);
		}
		allBalls.poses = poses;
		
		balls_pub_.publish(allBalls);

	}
	
	
	void PTScallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
				new pcl::PointCloud<pcl::PointXYZ>);
//		ROS_INFO("enter PTScallback");

		pcl::fromROSMsg(*cloudMsg, *cloud);
//		ROS_INFO("size = %d", (int) cloud->points.size());

		//	cloud.fields[]

		for (unsigned int i = 0; i <  circleList.size(); ++i) {
//			int pointId = getArrayId(circleList[i].y, circleList[i].x );
//			int x = cloud->points[pointId].x;
//			int y = cloud->points[pointId].y;
//			int z = cloud->points[pointId].z;
//			ROS_INFO("Ball %d,  (x,y,z) = (%d, %d, %d)",i, x, y, z);

		}

	}


	int getArrayId(int row, int col){
		return row*640 + col;
	}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;

	dynamic_reconfigure::Server < sensors_processing::TutorialsConfig > server;
	dynamic_reconfigure::Server<sensors_processing::TutorialsConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::spin();
	return 0;

}

int getClosestId(std::vector<CIRCLE> circles ){
	
	int retVal = -1;
	int minDist = 10000;
	
	for (unsigned int j = 0; j < circles.size(); ++j) {
		
		
		Point pt1 = Point(circles[j].x - circles[j].r, circles[j].y - circles[j].r);
		Point pt2 = Point(circles[j].x + circles[j].r, circles[j].y + circles[j].r);
//		ROS_INFO("draw circle %d (%d,%d) - (%d,%d)", j, pt1.x, pt1.y, pt2.x,pt2.y);

		//	okrąg wychodzi poza obrazek
		if (pt2.y > 479 || pt2.x > 639) {
			continue;
		}
		//	kwadrat w opisany na okręgu
//		rectangle(gray, pt1, pt2, 255);

		if(getDistance(circles[j]) < minDist){
			minDist = getDistance(circles[j]);
			retVal = j;
		}
	//	ROS_INFO("j=%d   dist=%d  minDist=%d", j, getDistance(circles[j]), minDist );
	}
	return retVal;
	
}

void drawAllCrircles(std::vector<CIRCLE> circles, Mat mat ){
	
	for (unsigned int j = 0; j < circles.size(); ++j) {
		
		Point pt1 = Point(circles[j].x - circles[j].r, circles[j].y - circles[j].r);
		Point pt2 = Point(circles[j].x + circles[j].r, circles[j].y + circles[j].r);
		//	okrąg wychodzi poza obrazek
		if (pt2.y > 479 || pt2.x > 639) {
			continue;
		}
		//	kwadrat w opisany na okręgu
		rectangle(mat, pt1, pt2, 255);

	}
}



/**
 * Metoda oblicza odleglosc okręgu circle od środka dołu obrazu.
 */
int getDistance(CIRCLE circle){

	int x = circle.x;
	int y = circle.y;

	double dist = sqrt( pow(320-x, 2) + pow(480-y, 2) );

	return (int)dist;
}
	

void ImageConverter::stopExplore(){
	publishAlgState("GO_TO_BALL");
	if(explore == false)
		return;

	ROS_INFO("stopExplore");

	explore = false;
	ac.cancelAllGoals ();
}

void ImageConverter::startExplore(){
	publishAlgState("SEARCH_BALLS");
	if(explore == true)
		return;

	ROS_INFO("startExplore");
	explore = true;
	ac.cancelAllGoals ();
}















/*
 if (contours.size()) {
 for (int i=0; i<contours.size(); i++) {
 drawContours( gray, contours[i], color, color, -1, CV_FILLED, 8 );


 }
 }*/

//   cvZero( gray );
/*
 for( ; contour != 0; contour = contour->h_next )
 {
 int color = CV_RGB( rand(), rand(), rand() );
 cvDrawContours( gray, contour, color, color, -1, CV_FILLED, 8 );
 }
 */

//    cvNamedWindow( "Components", 1 );
//    cvShowImage( "Components", dst );
// smooth it, otherwise a lot of false circles may be detected
/*

 vector<Vec3f> circles;
 HoughCircles(gray, circles, CV_HOUGH_GRADIENT,  2,  gray.rows/50, 200, 100, 10, 50);


 for( size_t i = 0; i < circles.size(); i++ )
 {
 ROS_INFO("Circle detected");
 Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
 int radius = cvRound(circles[i][2]);
 // draw the circle center
 circle( cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0 );
 circle( gray, center, 3, Scalar(0,255,0), -1, 8, 0 );
 // draw the circle outline
 circle( cv_ptr->image, center, radius, Scalar(255,255,255), 3, 8, 0 );
 circle( gray, center, radius, Scalar(255,255,255), 3, 8, 0 );
 }
 */










/*






	void imageCb1(const sensor_msgs::ImageConstPtr& msg) {
		ROS_INFO("I heard color image");

		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;

		}

		Mat gray;
		int count = 0;
		double area = 0.0;
		double area1 = 0.0;
		double x = 0.0;
		double y = 0.0;
		double maxArea = 0.0;

		cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

		Semaphore semaphore("test_sem");
		semaphore.lock();
//              ROS_INFO("imageCb semaphore in");

		int circleIndex = -1;

		for (unsigned int j = 0; j < circleList.size(); ++j) {

			Point pt1 = Point(circleList[j].x - circleList[j].r,
					circleList[j].y - circleList[j].r);
			Point pt2 = Point(circleList[j].x + circleList[j].r,
					circleList[j].y + circleList[j].r);
			ROS_INFO("draw circle %d (%d,%d) - (%d,%d)", j, pt1.x, pt1.y, pt2.x,
					pt2.y);

			if (pt2.y > 479 || pt2.x > 639) {
				continue;
			}

			Range rowRange = Range(pt1.y, pt2.y);
			Range colRange = Range(pt1.x, pt2.x);

			//              ROS_INFO("draw  %d rowRange (%d,%d) - colRange (%d,%d)",i, rowRange.start, rowRange.end, colRange.start, colRange.end);
			//              ROS_INFO("mat  %d rows %d - cols %d",i, gray.rows, gray.cols);

			Mat currMat = Mat(gray, rowRange, colRange).clone();

			int cols = currMat.cols;
			int rows = currMat.rows;

			int backGrnColor = ((int) currMat.at < uint8_t
					> (0, 0) + (int) currMat.at < uint8_t
					> (0, cols - 1) + (int) currMat.at < uint8_t
					> (rows - 1, 0) + (int) currMat.at < uint8_t
					> (rows - 1, cols - 1)) / 4;

			int xOffset = pt1.x;
			int yOffset = pt1.y;

			int ballColor = (int) currMat.at < uint8_t
					> ((int) rows / 2, (int) cols / 2);

			rectangle(gray, pt1, pt2, 255);

//			     if(abs(ballColor - backGrnColor) > 10){
//			 rectangle(gray, pt1, pt2, 255);
//			 continue;
//			 }

			int trashValue = (ballColor + backGrnColor) / 2;

			threshold(currMat, currMat, trashValue, 255, THRESH_BINARY);
			//      adaptiveThreshold(currMat, currMat, trashValue, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 5, 2.0);

			std::vector < std::vector<Point> > contours;
			findContours(currMat, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

			//      ROS_INFO("There are %d contours", contours.size());

			Point2f center(0.f, 0.f);
			float radius = 0.f;

			for (int i = 0; i < contours.size(); i++) {
				//      drawContours(gray, contours, i, 255);
				area = contourArea(contours[i]);
				//1030
				if (area < 2030 && area > 100) {

					Moments mom = moments(contours[i]);

					double M11 = mom.m11 - (mom.m10 * mom.m01) / mom.m00;
					double M02 = mom.m02 - (mom.m01 * mom.m01) / mom.m00;
					double M20 = mom.m20 - (mom.m10 * mom.m10) / mom.m00;

					// for circle it should be ~0.0063
					double M7 = (M20 * M02 - M11 * M11)
							/ (mom.m00 * mom.m00 * mom.m00 * mom.m00);

					// circle
					if (M7 < 0.007 && M7 > 0.0061) {

						//                   rectangle(gray, pt1, pt2, 0);

						//                      drawContours(cv_ptr->image, contours, i, CV_RGB(255, 0, 255));
						//                      drawContours(gray, contours, i, CV_RGB(255, 0, 255));

						double r = sqrt(area / 3.14);

						minEnclosingCircle(contours[i], center, radius);
						area1 = area;

						//circle(gray, center, r, CV_RGB(255, 0, 255));

						if (area > maxArea) {
							maxArea = area;
							x = center.x;
							y = center.y;
						}
						//      rectangle(gray, pt1, pt2, 255);

						//  ROS_INFO("%d   circle, M7 = %f , size = %f, odlegosc od kamery = %f, x=%f, y=%f",i,M7, area, 10.5/r, center.x, center.y);
						++count;

						circleIndex = j;

						//      ballList.push_back(new PosAndId(count, center.x, center.y));

					}
					// probably circle
					else if (M7 < 0.0085) {
						//                                      ROS_INFO("%d    probably circle, M7 = %f, area=%d",i,M7, area);

					}
					// there is small chance that it's circular, but show them too
					else if (M7 < 0.011) {
						//                                      ROS_INFO("%d    no circle, M7 = %f, area=%d",i,M7, area);
					}

					//      ROS_INFO("area0 = %f  area1 = %f", area0, area1);
					//      ROS_INFO("M7 = %f",M7);
				}
			}


		semaphore.unlock();

		cv_bridge::CvImage out_msg;
		out_msg.header = cv_ptr->header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
		out_msg.image = gray; // Your cv::Mat

		image_pub_.publish(out_msg.toImageMsg());

		//      image_pub_.publish(gray);

#ifdef _ON_PC_
		imshow(WINDOW_COLOR_1, gray);

		waitKey(3);
#endif
//              ROS_INFO("imageCb semaphore out");

	}







*/

