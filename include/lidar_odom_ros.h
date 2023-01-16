#pragma once

#include <mutex>
#include <ros/ros.h>

#include "state.h"
#include "scan_process.h"
#include "mapping.h"


class LidarOdomROS
{
public:
  LidarOdomROS();

  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg);

  void imuCB(const sensor_msgs::Imu::ConstPtr &msg_in);

  bool getSensorData(SensorData &sensor_data);

  void runCB(const ros::TimerEvent& e);

private:
  void pubOdom(const ros::Publisher &pub, const Esekf &kf, double time);

  void pubCloud(const ros::Publisher &pub_cloud, CloudPtr &cloud, double time);

  template <typename T> void setOdomMsg(T &out);

  ros::NodeHandle nh_;

  std::mutex mtx_;
  double last_timestamp_lidar_ = 0;
  double newest_imu_stamp_ = -1.0;
  double lidar_end_time_ = 0;

  std::deque<LidarData> scan_q_;
  std::deque<sensor_msgs::Imu::ConstPtr> imu_q_;
  bool scan_pub_ = false;
  boost::shared_ptr<ScanProcess> scan_process_;
  boost::shared_ptr<Esekf> kf_;
  boost::shared_ptr<Mapping> mapping_;
  ros::Subscriber sub_pcl_;
  ros::Subscriber sub_imu_;
  ros::Publisher pub_cloud_;
  ros::Timer run_timer_;
  State state_;
  //ros::Publisher pub_cloud2 = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_cmp", 100000);

  ros::Publisher pub_odom_;
};
