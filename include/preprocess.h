#pragma once

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "common.h"


enum TIME_UNIT
{
  SEC = 0,
  MS = 1,
  US = 2,
  NS = 3
};


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

class Preprocess
{
public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess(double blind, int time_unit, int point_filter_num);
  ~Preprocess();

  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, CloudPtr &pcl_out);

  Cloud cloud_; // cloud in lidar frame.
  int point_filter_num_, time_unit_;
  double blind_;
  double time_unit_scale_;

private:
  void velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);

};
