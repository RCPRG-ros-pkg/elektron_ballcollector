#include <ros/ros.h>
#include <joy/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Image.h>
#include <ros/console.h>

ros::NodeHandle n;

void imageCallback(const sensor_msgs::Image::ConstPtr& img) {
	ROS_INFO("I heard Image");

	ros::Publisher chatter_pub = n.advertise<sensor_msgs::Image>("piotrek_image_talker", 1);

	//printf("aaaa");

	int height = img->height;
	int width = img->width;

	sensor_msgs::Image msg;

	msg.header.seq = img->header.seq;
	msg.header.stamp = img->header.stamp;
	msg.header.frame_id = img->header.frame_id;

	msg.width = img->width;
	msg.height = img->height;
	msg.encoding = img->encoding;
	msg.is_bigendian = img->is_bigendian;
	msg.step = img->step;

	for(int i=0; i< msg.height; ++i){
		for(int j=0; j < msg.width; ++j){
			msg.data[i*msg.width + j] = img->data[i*msg.width + j];
		}
	}

	chatter_pub.publish(msg);

}


int main(int argc, char** argv) {
	ros::init(argc, argv, "piotrek_first_image_rec_node");

//	ros::init(argc, argv, "piotrek_image_talker");




	ros::Subscriber sub = n.subscribe<sensor_msgs::Image> ("/camera/depth/image", 10, imageCallback );



	ROS_DEBUG("init FirstImageRec");
/*

	 int count = 0;
	  while (ros::ok())
	  {


			int height = img->height;
			int width = img->width;


		//	ROS_INFO("\n\n\n\n");

		//	ROS_INFO("imageCallback width=[%d], height=[%d]",width, height );


			sensor_msgs::Image msg;

			msg.header.seq = img->header.seq;
			msg.header.stamp = img->header.stamp;
			msg.header.frame_id = img->header.frame_id;

			msg.width = img->width;
			msg.height = img->height;
			msg.encoding = img->encoding;
			msg.is_bigendian = img->is_bigendian;
			msg.step = img->step;

			for(int i=0; i< msg.height; ++i){
				for(int j=0; j < msg.width; ++j){
					msg.data[i*msg.width + j] = img->data[i*msg.width + j];
				}
			}

			chatter_pub.publish(msg);


	    ros::spinOnce();

	    loop_rate.sleep();
	    ++count;
	  }

*/

//	ros::Subscriber sub = n.subscribe<sensor_msgs::Image> ("/camera/depth/image", 10, imageCallback );




	ros::spin();
}
