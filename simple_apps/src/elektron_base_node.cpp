#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include "elektron.hpp"

ros::Time cmd_time;

Protonek *p;

void twistCallback(const geometry_msgs::TwistConstPtr& msg) {
double rotational_term = msg->angular.z * 0.335 / 2.0;
double rvel = -msg->linear.x + rotational_term;
double lvel = -msg->linear.x - rotational_term;

p->setVelocity(lvel, rvel);
}

int main(int argc, char** argv) {
ros::init(argc, argv, "elektron_base_node");
ros::NodeHandle n;
ros::NodeHandle nh("~");

bool publish_odom_tf;
bool dump;

double ls, rs;

ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry> ("odom", 1);

tf::TransformBroadcaster odom_broadcaster;

ros::Subscriber twist_sub = n.subscribe("cmd_vel", 1, &twistCallback);

ros::Rate loop_rate(100);

std::string dev;

if (!nh.getParam("device", dev)) {
dev = "/dev/protonek";
}

if (!nh.getParam("publish_odom_tf", publish_odom_tf)) {
publish_odom_tf = false;
}

if (!nh.getParam("dump", dump)) {
dump = false;
}

if (!nh.getParam("lin_scale", ls)) {
                ls = 1.0;
                        }
                        
                        if (!nh.getParam("rot_scale", rs)) {
                                        rs = 1.0;
                                                }
                                                
                                                nav_msgs::Odometry odom;
                                                odom.header.frame_id = "odom";
                                                odom.child_frame_id = "base_link";
                                                
                                                geometry_msgs::TransformStamped odom_trans;
                                                odom_trans.header.frame_id = "odom";
                                                odom_trans.child_frame_id = "base_link";
                                                
                                                // initialize hardware
                                                p = new Protonek(dev);
                                                
                                                if (p->isConnected()) {
                                                if (dump)
                                                p->dump();
                                                
                                                p->setParams(ls, rs);
                                                
                                                while (ros::ok()) {
                                                double x, y, th, xvel, thvel;
                                                
                                                ros::Time current_time = ros::Time::now();
                                                
                                                p->update();
                                                p->updateOdometry();
                                                p->getOdometry(x, y, th);
                                                p->getVelocity(xvel, thvel);
                                                
                                                //since all odometry is 6DOF we'll need a quaternion created from yaw
                                                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
                                                
                                                
                                                if (publish_odom_tf) {
                                                //first, we'll publish the transform over tf
                                                odom_trans.header.stamp = current_time;
                                                
                                                odom_trans.transform.translation.x = x;
                                                odom_trans.transform.translation.y = y;
                                                odom_trans.transform.translation.z = 0.0;
                                                odom_trans.transform.rotation = odom_quat;
                                                
                                                //send the transform
                                                odom_broadcaster.sendTransform(odom_trans);
                                                }
                                                
                                                //next, we'll publish the odometry message over ROS
                                                odom.header.stamp = current_time;
                                                
                                                //set the position
                                                odom.pose.pose.position.x = x;
                                                odom.pose.pose.position.y = y;
                                                odom.pose.pose.position.z = 0.0;
                                                odom.pose.pose.orientation = odom_quat;
                                                
                                                odom.pose.covariance[0] = 0.00001;
                                                odom.pose.covariance[7] = 0.00001;
                                                odom.pose.covariance[14] = 10.0;
                                                odom.pose.covariance[21] = 1.0;
                                                odom.pose.covariance[28] = 1.0;
                                                odom.pose.covariance[35] = thvel + 0.001;
                                                
                                                //set the velocity
                                                odom.child_frame_id = "base_link";
                                                odom.twist.twist.linear.x = xvel;
                                                odom.twist.twist.linear.y = 0.0;
                                                odom.twist.twist.angular.z = thvel;
                                                
                                                //publish the message
                                                odom_pub.publish(odom);
                                                
                                                ros::spinOnce();
                                                loop_rate.sleep();
                                                }
                                                } else {
                                                ROS_ERROR("Connection to device %s failed", dev.c_str());
                                                }
                                                
                                                return 0;
                                                }
                                                
                                                