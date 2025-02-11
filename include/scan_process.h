#pragma once

#include <sensor_msgs/PointCloud2.h>

#include "common.h"
#include <pcl/filters/voxel_grid.h>


namespace velodyne_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(uint16_t, ring, ring))

class ScanProcess
{
public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ScanProcess(double blind, int point_filter_num, double filter_size_surf_min);
  ~ScanProcess();

  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, CloudPtr &pcl_out);

  CloudPtr downsample(const CloudPtr &cloud);


  Cloud cloud_; // cloud in lidar frame.
  int point_filter_num_;
  double blind2_;

  pcl::VoxelGrid<PointType> downsampe_filter_;

private:
  void velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);

};
