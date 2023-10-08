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
  CloudPtr cloud(new Cloud());

  pcl::PointCloud<OusterPoint> cloud_orig;
  pcl::fromROSMsg(*msg, cloud_orig);
  // printf("base_stamp %f\n", base_stamp);
  int plsize = cloud_orig.points.size();
  if (plsize == 0)
    cloud_ = *cloud;
  for (int i = 0; i < plsize; i++)
  {
    if ((i / 64) % 8 == 0)
    {
      PointType added_pt;
      added_pt.x = cloud_orig.points[i].x;
      added_pt.y = cloud_orig.points[i].y;
      added_pt.z = cloud_orig.points[i].z;
      added_pt.intensity = cloud_orig.points[i].intensity;
      added_pt.time = cloud_orig.points[i].t * 1e-9;

      const double dist2 = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
      if (dist2 < 0 || dist2 > 200*200)
        continue;
      cloud->points.push_back(added_pt);
    }
  }
  std::sort(cloud->points.begin(), cloud->points.end(), time_cmp);
  cloud_ = *cloud;
}