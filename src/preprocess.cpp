#include "preprocess.h"

Preprocess::Preprocess(double blind, int time_unit, int point_filter_num)
    : blind_(blind),
      time_unit_(time_unit),
      point_filter_num_(point_filter_num){}

Preprocess::~Preprocess() {}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloud::Ptr &pcl_out)
{
  switch (time_unit_)
  {
  case SEC:
    time_unit_scale_ = 1.e3f;
    break;
  case MS:
    time_unit_scale_ = 1.f;
    break;
  case US:
    time_unit_scale_ = 1.e-3f;
    break;
  case NS:
    time_unit_scale_ = 1.e-6f;
    break;
  default:
    time_unit_scale_ = 1.f;
    break;
  }

  velodyneHandler(msg);

  *pcl_out = pl_;
}

void Preprocess::velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_.clear();

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0)
    return;

  for (int i = 0; i < plsize; i++)
  {

    if (i % point_filter_num_ == 0)
    {

      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.time = pl_orig.points[i].time * time_unit_scale_; // time unit: ms // std::cout<<added_pt.time<<std::endl;

      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind_ * blind_))
      {
        pl_.points.push_back(added_pt);
      }
    }
  }
}
