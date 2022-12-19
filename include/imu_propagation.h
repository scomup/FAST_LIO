#pragma once

#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "state.h"
#include "esekf.h"

#include "common_lib.h"

// Preconfiguration

#define MAX_INI_COUNT (10)


// IMU Process and undistortion
class IMUPropagation
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  IMUPropagation();
  ~IMUPropagation();
  
  void reset();
  void reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void setExtrinsic(const Vec3 &transl, const Mat3 &rot);
  void setExtrinsic(const Vec3 &transl);
  void setExtrinsic(const Mat4 &T);
  void setGyrCov(const Vec3 &scaler);
  void setAccCov(const Vec3 &scaler);
  void setGyrBiasCov(const Vec3 &b_g);
  void setAccBiasCov(const Vec3 &b_a);
  void process(const SensorData &sensor_data, ESEKF::Esekf &kf_state, PointCloud::Ptr &pcl_un_);

  Vec3 cov_acc_;
  Vec3 cov_gyr_;
  Vec3 cov_bias_gyr_;
  Vec3 cov_bias_acc_;
  double first_lidar_time_;
  Vec3 mean_acc_;
  Vec3 mean_gyr_;

 private:
  bool init(const SensorData &sensor_data, ESEKF::Esekf &kf_state);
  void undistortCloud(const SensorData &sensor_data, ESEKF::Esekf &kf_state, PointCloud &pcl_in_out);

  Eigen::Matrix<double, ESEKF::NZ, ESEKF::NZ> Q_;
  PointCloud::Ptr cur_pcl_un_;
  sensor_msgs::ImuConstPtr last_imu_;
  std::deque<sensor_msgs::ImuConstPtr> v_imu_;
  std::vector<ESEKF::BPInfo> imu_pose_;
  Mat3 Ril_;
  Vec3 til_;
  Vec3 gyr_last_;
  Vec3 acc_last_;
  double start_timestamp_;
  double last_lidar_end_time_;
  int    init_imu_num_ = 0;
  bool   imu_need_init_ = true;
};