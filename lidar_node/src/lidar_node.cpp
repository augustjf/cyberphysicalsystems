#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>


ros::Publisher pub;
bool first = true;
pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud(new pcl::PointCloud<pcl::PointXYZ>());



void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(0.5f, 0.5f, 0.5f);
  voxel_grid.setInputCloud(cloud);
  voxel_grid.filter(*cloud);

  if (first) {
    *previous_cloud = *cloud;
    first = false;
    return;
  }

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(previous_cloud);
  icp.setInputTarget(cloud);

  icp.setMaximumIterations(60);
  pcl::PointCloud<pcl::PointXYZ> final_cloud;
  icp.align(final_cloud);

  if (icp.hasConverged()) {
    std::cout << "ICP converged." << std::endl;
    Eigen::Matrix4f transformation = icp.getFinalTransformation();

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = "previous_cloud";
    transform_stamped.child_frame_id = "current_cloud";
    transform_stamped.header.stamp = msg->header.stamp;

    transform_stamped.transform.translation.x = transformation(0, 3);
    transform_stamped.transform.translation.y = transformation(1, 3);
    transform_stamped.transform.translation.z = transformation(2, 3);

    Eigen::Matrix3f rotation_matrix = transformation.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rotation_matrix);
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    pub.publish(transform_stamped);

    *previous_cloud = *cloud;
    
  } else {
    std::cout << "ICP did not converge." << std::endl;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/os1_cloud_node/points", 1, lidar_callback);
  pub = nh.advertise<geometry_msgs::TransformStamped>("/lidar_transform", 1);
  ros::spin();
  return 0;
}