#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <original_pointcloud_msgs/pub_pointcloud.h>
#include "d435.hpp"
#include "PointCloud.hpp"

enum Direction
{
  Front,
  Right,
  Left,
  Back,
};

using namespace original_pointcloud_msgs;

std::string getTimeNow()
{
  return boost::posix_time::to_iso_extended_string(
      ros::Time::now().toBoost());
}

pub_pointcloud toPubPointCloud(const PointCloud &pc)
{
  pub_pointcloud new_pc;
  sensor_msgs::PointCloud2 pcl_msg;
  auto pcl_pc = pc.get_cloud();
  pcl::toROSMsg(*pcl_pc, pcl_msg);

  new_pc.id = "omni_d435_01";
  new_pc.type = "omni_d435";
  new_pc.space = "real";
  new_pc.time = getTimeNow();
  new_pc.scan_start_time = "";

  new_pc.height = pcl_msg.height;
  new_pc.width = pcl_msg.width;
  new_pc.fields = pcl_msg.fields;
  new_pc.is_bigendian = pcl_msg.is_bigendian;
  new_pc.point_step = pcl_msg.point_step;
  new_pc.row_step = pcl_msg.row_step;
  new_pc.data = pcl_msg.data;
  new_pc.is_dense = pcl_msg.is_dense;

  return new_pc;
}

int main(int argc, char **argv)
try
{
  constexpr int num_of_using_device = 4;
  const std::string serial_numbers[4] = {
      "102422073987", "102422070478", "102122072472", "102422071935"};
  D435 *cameras[num_of_using_device];
  for (int i = 0; i < num_of_using_device; i++)
  {
    std::cout << "camera init:" << serial_numbers[i] << std::endl;
    cameras[i] = new D435(serial_numbers[i]);
    std::cout << "finish camera initing:" << std::endl;
  }

  std::cout << "Initialize ROS" << std::endl;
  ros::init(argc, argv, "omni_d435_pub");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<pub_pointcloud>("pointcloud", 1);

  while (ros::ok())
  {
    PointCloud pcs[4];
    for (int i = 0; i < num_of_using_device; i++)
    {
      std::cout << "start process of " << i << "'s camera " << std::endl;
      cameras[i]->update();
      auto points = cameras[i]->get_points();
      auto color = cameras[i]->get_color();
      pcs[i] = PointCloud(points, color);

      pcs[i].z_range_filter(0.0, 3.0);

      switch (i)
      {
      case Direction::Front:
        pcs[i].filter([](pcl::PointXYZRGB &p)
                      { p.z += 0.080; });
        break;
      case Direction::Back:
        pcs[i].filter([](pcl::PointXYZRGB &p)
                      {
                p.x=-p.x;
                p.z=-p.z;
                p.z-=0.080; });
        break;
      case Direction::Right:
        pcs[i].filter([](pcl::PointXYZRGB &p)
                      {
                float x=p.x;
                p.x=p.z;
                p.z=-x;
                p.x+=0.080; });
        break;
      case Direction::Left:
        pcs[i].filter([](pcl::PointXYZRGB &p)
                      {
                float x=p.x;
                p.x=-p.z;
                p.z=x;
                p.x-=0.080; });
        break;
      }
    }

    PointCloud merged;
    for (auto &p : pcs)
    {
      merged.extended(p);
    }

    auto pcl_msg=toPubPointCloud(merged);

    ROS_INFO("publish:%d", merged.get_cloud()->size());

    // pcl_msg.header.frame_id = "map";
    pub.publish(pcl_msg);

    ros::spinOnce();
  }

  return 0;
}
catch (const rs2::error &e)
{
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
}
catch (const std::exception &e)
{
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}