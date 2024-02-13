#pragma once

#include <iostream>
#include <algorithm>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>

// Intel Realsense Headers
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/io.h>
#include <pcl/filters/statistical_outlier_removal.h>

class PointCloud
{
public:
    using pc = pcl::PointCloud<pcl::PointXYZRGB>;
    using pc_ptr = pc::Ptr;
    PointCloud();
    PointCloud(const rs2::points &, const rs2::video_frame &);
    void save_to_pcd(const std::string &) const;
    pc_ptr get_cloud() const { return cloud; }
    void z_range_filter(float,float);
    void filter(void (*func)(pcl::PointXYZRGB &));
    void filter(pcl::PointXYZRGB &(*func)(const pcl::PointXYZRGB &));
    PointCloud extended(const PointCloud &);

private:
    static std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
    pc_ptr cloud;
};