#include <ros/ros.h>
#include <original_pointcloud_msgs/pub_pointcloud.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void pointcloudCallback(const original_pointcloud_msgs::pub_pointcloud &pc_msg)
{
  static int cnt = 1;

  sensor_msgs::PointCloud2 pc2_msg;
  pc2_msg.height = pc_msg.height;
  pc2_msg.width = pc_msg.width;
  pc2_msg.fields = pc_msg.fields;
  pc2_msg.is_bigendian = pc_msg.is_bigendian;
  pc2_msg.point_step = pc_msg.point_step;
  pc2_msg.row_step = pc_msg.row_step;
  pc2_msg.data = pc_msg.data;
  pc2_msg.is_dense = pc_msg.is_dense;

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(pc2_msg, cloud);
  // auto now = ros::Time::now().sec;
  auto filename = "/home/suzaku/Downloads/pcd/LICTiA/merged_labo" + std::to_string(cnt) + ".pcd";

  ROS_INFO("subscribe: %s(SIZE: %d)", filename.c_str(), cloud.size());
  pcl::io::savePCDFileBinary(filename, cloud);

  cnt++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_pcl_practice_sub");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("pointcloud", 10, pointcloudCallback);

  ros::spin();
  return 0;
}