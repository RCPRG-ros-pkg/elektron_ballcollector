#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>


using namespace cv;


namespace enc = sensor_msgs::image_encodings;

//static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);

 //   cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
   // cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	  ROS_INFO("I heard Image");

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

//    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
   //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    	Mat gray;

        cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
        // smooth it, otherwise a lot of false circles may be detected
        GaussianBlur( gray, gray, Size(15, 15), 2, 2 );
        vector<Vec3f> circles;
        HoughCircles(gray, circles, CV_HOUGH_GRADIENT,
                     2, gray.rows/4, 200, 100 );
        for( size_t i = 0; i < circles.size(); i++ )
        {
             Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
             int radius = cvRound(circles[i][2]);
             // draw the circle center
             circle( cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0 );
             // draw the circle outline
             circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }
    //    namedWindow( "circles", 1 );
    //    imshow( "circles", img );


    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
