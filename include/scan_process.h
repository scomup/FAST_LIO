#pragma once

#include <sensor_msgs/PointCloud2.h>

#include "common.h"


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

  ScanProcess(double blind, int point_filter_num);
  ~ScanProcess();

  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, CloudPtr &pcl_out);

  Cloud cloud_; // cloud in lidar frame.
  int point_filter_num_;
  double blind2_;

private:
  void velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);

};
