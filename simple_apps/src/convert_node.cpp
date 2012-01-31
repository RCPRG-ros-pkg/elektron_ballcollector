#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <math.h>

#include <geometry_msgs/Twist.h>

// 	maksymalny rozmiar piłeczki - mieści się w kwadracie o boku 37px, czyli jej promień to ok 14px, czyli rozmiar 14^2*pi = ok. 615px
//	maksymalny obwód to 2*pi*r =


//	z odległości 100cm piłeczka ma rozmiar ok 360px
//	z odległości 70cm piłeczka ma rozmiar ok 655px

using namespace cv;

namespace enc = sensor_msgs::image_encodings;



class ImageConverter {
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Publisher vel_pub_;
	int counter;

public:
	ImageConverter() :
			it_(nh_) {
		image_pub_ = it_.advertise("out", 1);
		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
				&ImageConverter::imageCb, this);

		vel_pub_ = nh_.advertise<geometry_msgs::Twist> ("cmd_vel", 1);

		counter = 0;

	}

	~ImageConverter() {

	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg) {
//	  ROS_INFO("I heard Image");

		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

//    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
		//   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

		Mat gray;

		cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
		//   cvtColor(cv_ptr->image, cont, CV_BGR2GRAY);

		GaussianBlur(gray, gray, Size(9, 9), 2, 2);

		threshold(gray, gray, 150, 255, THRESH_BINARY);

		//     GaussianBlur( gray, gray, Size(9, 9), 2, 2 );

		//       smooth( gray, gray, CV_GAUSSIAN, 11, 11 );

		Mat elem = (Mat_<int>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1);
		Point anchor = Point(-1, -1);

		dilate(gray, gray, elem, anchor, 1);

		erode(gray, gray, elem, anchor, 1);

		Mat countMap = gray.clone();

		std::vector < std::vector<Point> > contours;
		findContours(countMap, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

		//    ROS_INFO("There are %d contours", contours.size());


		Point2f center(0.f, 0.f);
		float radius =0.f;

		int count = 0;
		for (int i = 0; i < contours.size(); i++) {
			double area = contourArea(contours[i]);
			if (area < 1000 && area > 100 ) {

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
					//drawContours(gray, contours, i, CV_RGB(255, 0, 255));
					double r = sqrt(area/3.14);



					minEnclosingCircle(contours[i], center, radius );

					ROS_INFO("circle, M7 = %f , size = %f, odlegosc od kamery = %f, x=%f, y=%f",M7, area, 10.5/r, center.x, center.y);
					++count;
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
		geometry_msgs::Twist vel;
	//	vel.angular.z =
		vel.linear.x = 0.1;
		vel_pub_.publish(vel);*/

	//	++counter;
		if(count == 1 ){

			geometry_msgs::Twist vel;
			vel.angular.z = (320 - center.x)/500;
			vel.linear.x = (0.1 - (abs(320 - center.x))/3200)*(1 - pow(center.y/470, 3) );
			vel_pub_.publish(vel);
		}
		if(count == 0 ){
			geometry_msgs::Twist vel;
			vel.angular.z = 0.2;
			vel.linear.x = 0;
			vel_pub_.publish(vel);
		}

		ROS_INFO("There are %d balls", count);


		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}




