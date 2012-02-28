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




#include <dynamic_reconfigure/server.h>
#include <sensors_processing/TutorialsConfig.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>



// 	maksymalny rozmiar piłeczki - mieści się w kwadracie o boku 37px, czyli jej promień to ok 14px, czyli rozmiar 14^2*pi = ok. 615px
//	maksymalny obwód to 2*pi*r =


//	z odległości 100cm piłeczka ma rozmiar ok 360px
//	z odległości 70cm piłeczka ma rozmiar ok 655px


using ecl::Semaphore;

typedef struct myCircle{
	uint16_t x;
	uint16_t y;
	uint16_t r;
}CIRCLE;

std::vector<CIRCLE> circleList;


double val1 = 0.0;
double val2 = 0.0;
int morphIterations = 1;


/*void printGaussianPoiont(Mat &image, int x, int y, int r ){
	for(int i=x-r; i<x+r; ++i){
		for(int j=y-r; j<y+r; ++j){
			image.at<uint16_t>(y, x) = 127;
		}
	}
}*/


void callback(sensors_processing::TutorialsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f",
            config.double_param1, config.double_param2);

  val1 = config.double_param1;
  val2 = config.double_param2;
  morphIterations = config.morphIterations;
}

using namespace cv;

namespace enc = sensor_msgs::image_encodings;


bool fristLoop = true;
Mat firstImage;
Mat out_image;


#ifdef _ON_PC_

static const char WINDOW_GREY[] = "Gray";
static const char WINDOW_ORG[] = "Orginal";
static const char WINDOW_COUNT[] = "Cpunture";
static const char WINDOW_OUT[]	="Out";

static const char WINDOW_COLOR_1[]	="Image";

#endif

class ImageConverter {
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	image_transport::Subscriber image_depth_sub_;
	ros::Publisher vel_pub_;
	int counter;

public:
	ImageConverter() :
			it_(nh_) {
		image_pub_ = it_.advertise("out", 1);
		image_depth_sub_ = it_.subscribe("/camera/depth_registered/image_raw", 1, &ImageConverter::depthImageCb, this);

		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);

		vel_pub_ = nh_.advertise<geometry_msgs::Twist> ("cmd_vel", 1);

		counter = 0;

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


	void imageCb(const sensor_msgs::ImageConstPtr& msg) {
		  ROS_INFO("I heard color image");

		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;

		}

		Mat gray;

		cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

		Semaphore semaphore("test_sem");
		semaphore.lock();
		ROS_INFO("imageCb semaphore in");

		for(int i=0; i< circleList.size(); ++i ){


			 Point pt1 = Point(circleList[i].x -circleList[i].r, circleList[i].y - circleList[i].r);
			 Point pt2 = Point(circleList[i].x +circleList[i].r, circleList[i].y + circleList[i].r);
			 ROS_INFO("draw circle %d (%d,%d) - (%d,%d)",i, pt1.x, pt1.y, pt2.x, pt2.y);
			 rectangle(gray, pt1, pt2, 255);
		}



		semaphore.unlock();

		cv_bridge::CvImage out_msg;
		out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
		out_msg.image    = gray; // Your cv::Mat

		image_pub_.publish(out_msg.toImageMsg());

	//	image_pub_.publish(gray);

#ifdef _ON_PC_
		imshow(WINDOW_COLOR_1, gray);

		waitKey(3);
#endif
		ROS_INFO("imageCb semaphore out");


	}

	void depthImageCb(const sensor_msgs::ImageConstPtr& msg) {
	  ROS_INFO("I heard depth image");

		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_16UC1 );
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;

		}



		Mat gray, trash, orginal;

		cv_ptr->image.convertTo(gray, 	 CV_8UC1, 1./16);
		cv_ptr->image.convertTo(orginal, CV_16UC1, 1);

		cv_ptr->image.convertTo(gray, 	 CV_8UC1, 1./16);


		if(fristLoop){
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


		threshold(gray, trash, 190, 255, THRESH_BINARY);


		uint16_t pixVal = orginal.at<uint16_t>(200, 320);

		ROS_INFO("(320, 200)=  %d ", (int)pixVal);

		//orginal.at<uint16_t>(200, 320)= 60000;


		pixVal = orginal.at<uint16_t>(400, 320);

		ROS_INFO("(320, 400) =  %d ", (int)pixVal);


		std::vector < std::vector<Point> > contours;
	//	findContours(countMap, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	//	GaussianBlur(gray, gray, Size(9, 9), 2, 2);

		//	val1 = 17.85
		//	val2 = 7.65

		Canny( gray, gray, val1, val2 , 3);

		Mat elem = (Mat_<int>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1);
		Point anchor = Point(-1, -1);

		dilate(gray, gray, elem, anchor, 1);

//		cv.MorphologyEx(image, image5, temp, element, cv.CV_MOP_CLOSE, 2);

		Mat countMap = gray.clone();



		morphologyEx(countMap, countMap, MORPH_CLOSE, elem, anchor, morphIterations );

	//	InputArray src, OutputArray dst, int op, InputArray element, Point anchor=Point(-1,-1), int iterations=1,
	//	int borderType=BORDER_CONSTANT, const Scalar& borderValue=morphologyDefaultBorderValue()

	//	ROS_INFO("There are %d contours", contours.size());

		Mat empty = gray.clone();
		threshold(gray, empty, 255, 255, THRESH_BINARY);

		findContours(countMap, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	//	ROS_INFO("There are %d contours\n", contours.size());



		 for (int i=0; i<contours.size(); i++) {
		//	 drawContours( gray, contours[i], color, color, -1, CV_FILLED, 8 );
			 int area = contourArea(contours[i]);



			 Point pt1 = Point(639, 479);
			 Point pt2 = Point(0, 0);

			 for(int j=0; j<contours[i].size(); ++j){
				 pt1.x = min(pt1.x, contours[i][j].x );
				 pt1.y = min(pt1.y, contours[i][j].y );

				 pt2.x = max(pt2.x, contours[i][j].x );
				 pt2.y = max(pt2.y, contours[i][j].y );
			 }

			 int a = (pt2.x - pt1.x);

			 if(a < 10 || a > 40){
				 continue;
			 }
			 int center_y = (pt1.y + pt2.y)/2;
			 if(center_y < 150){
				 continue;
			 }
	//		 ROS_INFO("area %d = %d", i, area);
			 pt2.y = pt1.y + a;
			 rectangle(empty, pt1, pt2, 255);
			 drawContours(empty, contours, i, 255);
			// rectangle(gray, pt1, pt2, 255, CV_FILLED);
			// rectangle(out_image, pt1, pt2, 255, CV_FILLED);


	//		 Point center((pt1.x + pt2.x)/2, (pt1.y + pt2.y)/2);
	//		 circle(out_image,  center, a/2, 255, CV_FILLED);

			 for(int x=pt1.x; x<pt2.x; ++x){
			 		for(int y=pt1.y; y<pt2.y; ++y){

			 			out_image.at<uint8_t>(y, x) =
			 					out_image.at<uint8_t>(y, x) < 200 ?
			 							out_image.at<uint8_t>(y, x)+50
			 						    :out_image.at<uint8_t>(y, x) ;
			 		}
			 	}


		 }

		threshold(out_image, trash, 75, 255, THRESH_BINARY);

		Mat out_image_copy = out_image.clone();

		findContours(out_image_copy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

		Semaphore semaphore("test_sem");
		semaphore.lock();
		ROS_INFO("imageDepthCb semaphore in");

		circleList.clear();

		 for (int i=0; i<contours.size(); i++) {

			 Point pt1 = Point(639, 479);
			 Point pt2 = Point(0, 0);

			 for(int j=0; j<contours[i].size(); ++j){
				 pt1.x = min(pt1.x, contours[i][j].x );
				 pt1.y = min(pt1.y, contours[i][j].y );

				 pt2.x = max(pt2.x, contours[i][j].x );
				 pt2.y = max(pt2.y, contours[i][j].y );
			 }

			 int a = (pt2.x - pt1.x);
			 int center_y = (pt1.y + pt2.y)/2;

			 CIRCLE circle;
			 circle.r = a/2;

			 if(circle.r < 10 || circle.r > 20){
							 continue;
			 }

			 circle.x = pt1.x + circle.r;
			 circle.y = pt1.y + circle.r;
			 circleList.push_back(circle);
			 ROS_INFO("push back circle, vector size = %d",circleList.size());

			 /*
			 msg = Int16MultiArray()
			 msg.layout.dim = [MultiArrayDimension(signal, 1, 1)]
			 msg.data = datadict[signal]*/

		 }
		ROS_INFO("imageDepthCb semaphore out");
		 semaphore.unlock();


#ifdef _ON_PC_

//		imshow(WINDOW_COUNT, trash);

//		imshow(WINDOW_GREY, gray);
		//  waitKey(3);

//		imshow(WINDOW_ORG, countMap);

		imshow(WINDOW_OUT, out_image);

		waitKey(3);

#endif

	//	image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;


	  dynamic_reconfigure::Server<sensors_processing::TutorialsConfig> server;
	  dynamic_reconfigure::Server<sensors_processing::TutorialsConfig>::CallbackType f;

	  f = boost::bind(&callback, _1, _2);
	  server.setCallback(f);


	ros::spin();
	return 0;


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


