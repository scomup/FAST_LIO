#pragma once

#include "state.h"

MatNN processNoiseCov();

// Error State Extended Kalman Filter
class Esekf
{
public:
  using HFunc = std::function<bool (HData &, State &, CloudPtr &)>;

  Esekf(const double R, const int maximum_iter, const HFunc h_model);

  State getState() const;

  MatSS getP() const;

  void setState(const State &state);

  void setP(const MatSS &input_cov);

  // Forward Propagation  III-C
  void propagation(double &dt, MatNN &Q, const InputU &i_in);

  // update
  void iteratedUpdate(CloudPtr &cloud_ds);

  void setExtrinsic(const Vec3 &transl, const Mat3 &rot);

  void setGyrCov(const Vec3 &cov);

  void setAccCov(const Vec3 &cov);

  void setGyrBiasCov(const Vec3 &b_g);

  void setAccBiasCov(const Vec3 &b_a);

  void predict(const SensorData &sensor_data);

  CloudPtr undistortCloud(const SensorData &sensor_data);

private:
  VecS f_func(const State &state, const InputU &u, double dt) const;

  MatSS df_dx_func(const State &state, const InputU &u, double dt) const;

  MatSN df_dw_func(const State &state, const InputU &u, double dt) const;

  bool initImu(const SensorData &sensor_data);


  Vec3 cov_acc_;
  Vec3 cov_gyr_;
  Vec3 cov_bias_gyr_;
  Vec3 cov_bias_acc_;
  Vec3 mean_acc_;
  Vec3 mean_gyr_;


  CloudPtr cur_pcl_un_;

  sensor_msgs::ImuConstPtr last_imu_;

  std::vector<IMUPose> imu_pose_;

  Mat3 Ril_;
  Vec3 til_;
  Vec3 gyr_last_;
  Vec3 acc_last_;
  double start_timestamp_;
  double last_lidar_end_time_;
  int    init_imu_num_ = 0;
  bool   imu_need_init_ = true;

  State x_;
  MatSS P_;
  MatNN Q_;
  const double R_inv_;
  const int maximum_iter_;
  const HFunc h_model_;
  const double epsi_ = 0.001;
};
