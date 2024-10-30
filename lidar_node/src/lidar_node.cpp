#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_INFO("I heard: ");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/ouster/points", 1000, lidar_callback);
  ros::spin();
  return 0;
}