#include "preprocess.h"


Preprocess::Preprocess()
    : blind(0.01), point_filter_num(1)
{
}

Preprocess::~Preprocess() {}

void Preprocess::set(double bld, int pfilt_num)
{
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloud::Ptr &pcl_out)
{
  switch (time_unit)
  {
  case SEC:
    time_unit_scale = 1.e3f;
    break;
  case MS:
    time_unit_scale = 1.f;
    break;
  case US:
    time_unit_scale = 1.e-3f;
    break;
  case NS:
    time_unit_scale = 1.e-6f;
    break;
  default:
    time_unit_scale = 1.f;
    break;
  }

  velodyne_handler(msg);

  *pcl_out = pl_;
}

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_.clear();

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0)
    return;

  for (int i = 0; i < plsize; i++)
  {

    if (i % point_filter_num == 0)
    {

      PointType added_pt;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale; // curvature unit: ms // std::cout<<added_pt.curvature<<std::endl;

      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind))
      {
        pl_.points.push_back(added_pt);
      }
    }
  }
}
