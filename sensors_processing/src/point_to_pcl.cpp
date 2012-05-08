#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/transport_hints.h>

template<typename T> struct DepthTraits {};

template<>
struct DepthTraits<uint16_t>
{
  static inline bool valid(uint16_t depth) { return depth != 0; }
  static inline float toMeters(uint16_t depth) { return depth * 0.001f; } // originally mm
  static inline uint16_t fromMeters(float depth) { return (depth * 1000.0f) + 0.5f; }
  static inline void initializeBuffer(std::vector<uint8_t>& buffer) {} // Do nothing - already zero-filled
};

template<>
struct DepthTraits<float>
{
  static inline bool valid(float depth) { return std::isfinite(depth); }
  static inline float toMeters(float depth) { return depth; }
  static inline float fromMeters(float depth) { return depth; }

  static inline void initializeBuffer(std::vector<uint8_t>& buffer)
  {
    float* start = reinterpret_cast<float*>(&buffer[0]);
    float* end = reinterpret_cast<float*>(&buffer[0] + buffer.size());
    std::fill(start, end, std::numeric_limits<float>::quiet_NaN());
  }
};





namespace enc = sensor_msgs::image_encodings;

class PointToPCL{
	
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;

	image_transport::Subscriber image_depth_sub_;
	ros::Subscriber cam_info_sub_;
	ros::Subscriber all_balls;
	ros::Publisher all_balls_pub;


	image_geometry::PinholeCameraModel model_;
	sensor_msgs::Image depth_image_;

	bool depthImageCbInvoked;
	bool camInfoCbInvoked;


public:
	PointToPCL() :
		it_(nh_) {
		cam_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo> ("camera_info", 10, &PointToPCL::camInfoCb, this);
		image_depth_sub_ = it_.subscribe("image_depth", 1, &PointToPCL::depthImageCb, this);
		all_balls = nh_.subscribe<geometry_msgs::PoseArray> ("allBalls", 10, &PointToPCL::allBallsCb, this);

		all_balls_pub = nh_.advertise<geometry_msgs::PoseArray> ("allBallsXYZ", 1);

		depthImageCbInvoked = false;
		camInfoCbInvoked = false;
	}

	~PointToPCL() {
	}

	  void camInfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg);
	  void depthImageCb(const sensor_msgs::ImageConstPtr& msg);
	  void allBallsCb(const geometry_msgs::PoseArrayConstPtr& all_balls_msg);
	  // Handles float or uint16 depths
	  template<typename T>
	  void convert(const std::vector<geometry_msgs::Pose> poses);


};


int main(int argc, char** argv) {
	ros::init(argc, argv, "pointToPcl");
	PointToPCL p_t_pcl_;

	ros::spin();
	return 0;

}


void PointToPCL::camInfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	ROS_INFO("enter camInfoCb");
	camInfoCbInvoked = true;

	  // Update camera model
	  model_.fromCameraInfo(info_msg);

}

void PointToPCL::depthImageCb(const sensor_msgs::ImageConstPtr& msg) {
	ROS_INFO("enter depthImageCb");
	depthImageCbInvoked = true;
	depth_image_ = *msg;

	for(int i = 0; i < depth_image_.width*depth_image_.height; ++i){
		depth_image_.data[i] = msg->data[i];
	}

	ROS_INFO("leave depthImageCb");
}

void PointToPCL::allBallsCb(const geometry_msgs::PoseArrayConstPtr& all_balls_msg){
	ROS_INFO("enter allBallsCb");

	if(depthImageCbInvoked == false || camInfoCbInvoked == false){
		ROS_INFO("leave allBallsCb, data not ready");
		return;
	}

	  if (depth_image_.encoding == enc::TYPE_16UC1)
	  {
	    convert<uint16_t>(all_balls_msg->poses);
	  }
	  else if (depth_image_.encoding == enc::TYPE_32FC1)
	  {
	    convert<float>(all_balls_msg->poses);
	  }

}



template<typename T>
void PointToPCL::convert( const std::vector<geometry_msgs::Pose> poses)
{

  // Use correct principal point from calibration
  float center_x = model_.cx();
  float center_y = model_.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / model_.fx();
  float constant_y = unit_scaling / model_.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN();



//  ROS_INFO("center_x = %f, center_y = %f, constant_x = %f, constant_y = %f",center_x, center_y, constant_x, constant_y);
//	ROS_INFO("image height = %d", depth_image_.height);



  std::vector<geometry_msgs::Pose> posesXYZ;

  for (unsigned int i=0; i < poses.size(); i++){

	  int img_x = poses[i].position.x;
	  int img_y = poses[i].position.y;
	  int depth = poses[i].position.z;
	 // float depth = depth_image_.data[img_y*depth_image_.width +  img_x];

	  ROS_INFO("img_x = %d, img_y = %d, depth = %d",img_x, img_y, depth);
	  ROS_INFO("center_x = %f, center_y = %f, constant_x = %f,  constant_x=%f",center_x, center_y, constant_x, constant_y);

	  geometry_msgs::Pose pose;
	  pose.position.x =	 (img_x - center_x) * depth * constant_x;
	  pose.position.y = (img_y - center_y) * depth * constant_y;
	  pose.position.z = DepthTraits<T>::toMeters( depth );

/*	  pose.orientation.x = 0.0721103;
	  pose.orientation.y = -0.239501;
	  pose.orientation.z = -0.126247;
	  pose.orientation.w = 0.959948;*/

	  pose.orientation.x = 0.0;
	  pose.orientation.y = 0.0;
	  pose.orientation.z = 0.0;
	  pose.orientation.w = 1.0;


	  ROS_INFO("(x, y, z) = (%f, %f, %f)",pose.position.x, pose.position.y,  pose.position.z );

	  posesXYZ.push_back(pose);
  }

  geometry_msgs::PoseArray allBalls;
  allBalls.poses = posesXYZ;
  allBalls.header.frame_id = depth_image_.header.frame_id;
  all_balls_pub.publish(allBalls);



//  for() 	//	iteracja po wyznaczonych punktach




  /*
  PointCloud::iterator pt_iter = cloud_msg->begin();
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
  {
    for (int u = 0; u < (int)cloud_msg->width; ++u)
    {
      pcl::PointXYZ& pt = *pt_iter++;
      T depth = depth_row[u];

      // Missing points denoted by NaNs
      if (!DepthTraits<T>::valid(depth))
      {
        pt.x = pt.y = pt.z = bad_point;
        continue;
      }

      // Fill in XYZ
      pt.x = (u - center_x) * depth * constant_x;
      pt.y = (v - center_y) * depth * constant_y;
      pt.z = DepthTraits<T>::toMeters(depth);
    }
  }
  */
}

