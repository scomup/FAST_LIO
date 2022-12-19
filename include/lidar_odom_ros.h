#pragma once

#include "mapping.h"
#include "preprocess.h"
#include <algorithm>
#include "state.h"

class LidarOdomROS
{

public:
  LidarOdomROS();

  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg);

  void imuCB(const sensor_msgs::Imu::ConstPtr &msg_in);

  bool syncData(SensorData &sensor_data);

  void runCB(const ros::TimerEvent& e);

private:
  void pubOdom(const ros::Publisher &pub, const Esekf &kf, double time);

  void pubCloud(const ros::Publisher &pub_cloud, PointCloud::Ptr &cloud, double time);


  template <typename T> void setOdomMsg(T &out);

  std::mutex mtx_;
  ros::NodeHandle nh_;
  double last_timestamp_lidar_ = 0;
  double newest_imu_stamp_ = -1.0;
  double lidar_end_time_ = 0;

  std::deque<double> time_buffer_;
  std::deque<PointCloud::Ptr> lidar_buffer_;
  std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;
  bool scan_pub_ = false;
  boost::shared_ptr<Preprocess> p_pre_;
  boost::shared_ptr<Esekf> kf_;
  boost::shared_ptr<Mapping> mapping_;
  ros::Subscriber sub_pcl_;
  ros::Subscriber sub_imu_;
  ros::Publisher pub_cloud_;
  pcl::VoxelGrid<PointType> downsampe_filter_;
  ros::Timer run_timer_;
  State state_;
  //ros::Publisher pub_cloud2 = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_cmp", 100000);

  ros::Publisher pub_odom_;
  bool flg_first_scan_ = true;
};
