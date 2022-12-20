#include "preprocess.h"

const bool time_cmp(PointType &x, PointType &y) { return (x.time < y.time); };


Preprocess::Preprocess(double blind, int time_unit, int point_filter_num)
    : blind_(blind),
      time_unit_(time_unit),
      point_filter_num_(point_filter_num){}

Preprocess::~Preprocess() {}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloud::Ptr &cloud_out)
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

  *cloud_out = cloud_;
}

void Preprocess::velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  cloud_.clear();

  pcl::PointCloud<velodyne_ros::Point> cloud_orig;
  pcl::fromROSMsg(*msg, cloud_orig);
  int plsize = cloud_orig.points.size();
  if (plsize == 0)
    return;

  for (int i = 0; i < plsize; i++)
  {

    if (i % point_filter_num_ == 0)
    {

      PointType added_pt;
      added_pt.x = cloud_orig.points[i].x;
      added_pt.y = cloud_orig.points[i].y;
      added_pt.z = cloud_orig.points[i].z;
      added_pt.intensity = cloud_orig.points[i].intensity;
      added_pt.time = cloud_orig.points[i].time;
      //* time_unit_scale_; // time unit: ms // std::cout<<added_pt.time<<std::endl;
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind_ * blind_))
      {
        cloud_.points.push_back(added_pt); 
      }
    }
  }
  std::sort(cloud_.points.begin(), cloud_.points.end(), time_cmp);
}
