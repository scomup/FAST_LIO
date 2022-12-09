#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloud;

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

  Preprocess();
  ~Preprocess();

  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloud::Ptr &pcl_out);
  void set(double bld, int pfilt_num);

  PointCloud pl_; // cloud in lidar frame.
  int point_filter_num, time_unit;
  double blind;
  double time_unit_scale;

private:
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

};
