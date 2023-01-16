#include <pcl_conversions/pcl_conversions.h>

#include "scan_process.h"

const bool time_cmp(PointType &x, PointType &y) { return (x.time < y.time); };

ScanProcess::ScanProcess(double blind, int point_filter_num, double filter_size_surf_min)
    : blind2_(blind * blind),
      point_filter_num_(point_filter_num)
{
  downsampe_filter_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
}

ScanProcess::~ScanProcess() {}

CloudPtr ScanProcess::downsample(const CloudPtr &cloud)
{
  CloudPtr cloud_ds(new Cloud());
  downsampe_filter_.setInputCloud(cloud);
  downsampe_filter_.filter(*cloud_ds);
  return cloud_ds;
}

void ScanProcess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, CloudPtr &cloud_out)
{

  velodyneHandler(msg);

  *cloud_out = cloud_;
}

void ScanProcess::velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
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
      
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind2_))
      {
        cloud_.points.push_back(added_pt); 
      }
    }
  }
  std::sort(cloud_.points.begin(), cloud_.points.end(), time_cmp);
}
