#include "ikd_Tree.h"
#include "so3_math.h"
#include "esekf.h"
#include "state.h"
#include <ros/assert.h>

Esekf::Esekf(const double R, const int maximum_iter, const HFunc h_model)
    : R_inv_(1. / R),
      maximum_iter_(maximum_iter),
      h_model_(h_model),
      P_(MatSS::Identity()),
      imu_need_init_(true)
{
  init_imu_num_ = 0;
  Q_ = processNoiseCov();
  cov_acc_ = Vec3(0.1, 0.1, 0.1);
  cov_gyr_ = Vec3(0.1, 0.1, 0.1);
  cov_bias_gyr_ = Vec3(0.0001, 0.0001, 0.0001);
  cov_bias_acc_ = Vec3(0.0001, 0.0001, 0.0001);
  gyr_last_ = Vec3::Zero();
  til_ = Vec3::Zero();
  Ril_ = Mat3::Identity();
  last_imu_.reset(new sensor_msgs::Imu());
  mean_acc_ = Vec3::Zero();
  mean_gyr_ = Vec3::Zero();
};

// paper (2) f(x, u): kinematic model
// x: state
// u: input
// update state by input(IMU)
VecS Esekf::f_func(const State &state, const InputU &u, double dt) const
{
  VecS f = VecS::Zero();
  f.segment<3>(L_P) = state.vel;                                   // (7) row 2: velocity
  f.segment<3>(L_R) = u.gyr - state.bg;                            // (7) row 1: omega
  f.segment<3>(L_V) = state.rot * (u.acc - state.ba) + state.grav; // (7) row 3: acceleration
  return f * dt;
}
// paper (7) df_fx:
// Partial derivatives of the kinematic model(f) with respect to current state.
MatSS Esekf::df_dx_func(const State &s, const InputU &u, double dt) const
{
  MatSS df_dx = MatSS::Identity();
  df_dx.block<3, 3>(L_P, L_V) = Eigen::Matrix3d::Identity() * dt; // paper (7) Fx(2,3)
  // df_dx.block<3, 3>(L_R, L_R) = SO3Expmap(-(u.gyr - s.bg) * dt); // paper (7) Fx(1,1)
  df_dx.block<3, 3>(L_R, L_Bw) = -Eigen::Matrix3d::Identity() * dt;       // paper(7) Fx(1,4)
  df_dx.block<3, 3>(L_V, L_R) = -s.rot * skewSymMat((u.acc - s.ba)) * dt; // paper(7) Fx(3,1)
  df_dx.block<3, 3>(L_V, L_Ba) = -s.rot * dt;                             // paper(7) Fx(3,5)
  df_dx.block<3, 3>(L_V, L_G) = Eigen::Matrix3d::Identity() * dt;         // paper(7) Fx(3,6)
  return df_dx;
}

// paper (7) df_fw:
// Partial derivatives of the kinematic model(f) with respect to noise.
MatSN Esekf::df_dw_func(const State &s, const InputU &u, double dt) const
{
  MatSN df_dw = MatSN::Zero();
  df_dw.block<3, 3>(L_R, L_Nw) = -Eigen::Matrix3d::Identity() * dt;  //  paper (7) Fw(1,1)
  df_dw.block<3, 3>(L_V, L_Na) = -s.rot * dt;                        //  paper (7) Fw(3,2)
  df_dw.block<3, 3>(L_Bw, L_Nbw) = Eigen::Matrix3d::Identity() * dt; //  paper (7) Fw(4,3)
  df_dw.block<3, 3>(L_Ba, L_Nba) = Eigen::Matrix3d::Identity() * dt; //  paper (7) Fw(5,4)
  return df_dw;
}

State Esekf::getState() const
{
  return x_;
}

MatSS Esekf::getP() const
{
  return P_;
}

void Esekf::setState(const State &state)
{
  x_ = state;
}

void Esekf::setP(const MatSS &input_cov)
{
  P_ = input_cov;
}

void Esekf::setExtrinsic(const Vec3 &t, const Mat3 &R)
{
  til_ = t;
  Ril_ = R;
}

void Esekf::setGyrCov(const Vec3 &cov_gyr)
{
  cov_gyr_ = cov_gyr;
}

void Esekf::setAccCov(const Vec3 &cov_acc)
{
  cov_acc_ = cov_acc;
}

void Esekf::setGyrBiasCov(const Vec3 &b_g)
{
  cov_bias_gyr_ = b_g;
}

void Esekf::setAccBiasCov(const Vec3 &b_a)
{
  cov_bias_acc_ = b_a;
}

bool Esekf::initImu(const SensorData &sensor_data)
{
  // initializing ESEKF state
  for (const auto &imu : sensor_data.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &imu_gyr = imu->angular_velocity;
    mean_acc_ += Vec3(imu_acc.x, imu_acc.y, imu_acc.z);
    mean_gyr_ += Vec3(imu_gyr.x, imu_gyr.y, imu_gyr.z);
    init_imu_num_++;
  }

  if (init_imu_num_ > INIT_IMU_NUM)
  {
    mean_acc_ /= init_imu_num_;
    mean_gyr_ /= init_imu_num_;
    x_.grav = -mean_acc_ / mean_acc_.norm() * GRAVITY;
    x_.bg = mean_gyr_;
    x_.til = til_;
    x_.Ril = Eigen::Quaterniond(Ril_);
    P_.setIdentity();
    P_.block<3, 3>(L_Rli, L_Rli) = 0.00001 * Eigen::Matrix3d::Identity();
    P_.block<3, 3>(L_Tli, L_Tli) = 0.00001 * Eigen::Matrix3d::Identity();
    P_.block<3, 3>(L_Bw, L_Bw) = 0.0001 * Eigen::Matrix3d::Identity();
    P_.block<3, 3>(L_Ba, L_Ba) = 0.001 * Eigen::Matrix3d::Identity();
    P_.block<3, 3>(L_G, L_G) = 0.00001 * Eigen::Matrix3d::Identity();

    last_imu_ = sensor_data.imu.back();
    Q_.block<3, 3>(L_Nw, L_Nw).diagonal() = cov_gyr_;
    Q_.block<3, 3>(L_Na, L_Na).diagonal() = cov_acc_;
    Q_.block<3, 3>(L_Nbw, L_Nbw).diagonal() = cov_bias_gyr_;
    Q_.block<3, 3>(L_Nba, L_Nba).diagonal() = cov_bias_acc_;


    return true;
  }
  return false;
}

void Esekf::undistortCloud(const SensorData &sensor_data, Cloud &cloud)
{
  // add the imu of the last frame-tail to the of current frame-head
  auto imus = sensor_data.imu;
  imus.push_front(last_imu_);
  const double &imu_end_time = imus.back()->header.stamp.toSec();
  const double &cloud_end_time = sensor_data.stamp;

  // sort point clouds by offset time
  cloud = *(sensor_data.cloud);

  // Initialize IMU pose
  imu_pose_.clear();
  imu_pose_.push_back(IMUPose(0.0, acc_last_, gyr_last_, x_.vel, x_.pos, x_.rot));

  double dt = 0;

  InputU u;
  for (auto it = imus.begin(); it < (imus.end() - 1); it++)
  {
    auto &&head = *(it);
    auto &&tail = *(it + 1);

    if (tail->header.stamp.toSec() < last_lidar_end_time_)
      continue;

    Vec3 gyr(0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
             0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
             0.5 * (head->angular_velocity.z + tail->angular_velocity.z));
    Vec3 acc(0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
             0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
             0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z));

    if (head->header.stamp.toSec() < last_lidar_end_time_)
    {
      dt = tail->header.stamp.toSec() - last_lidar_end_time_;
    }
    else
    {
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }

    u.acc = acc;
    u.gyr = gyr;
    predict(dt, Q_, u);

    /* save the poses at each IMU measurements */
    gyr_last_ = gyr - x_.bg;
    acc_last_ = x_.rot * (acc - x_.ba);

    acc_last_ += x_.grav;

    double &&offset_time = tail->header.stamp.toSec() - cloud_end_time;
    imu_pose_.push_back(IMUPose(offset_time, acc_last_, gyr_last_, x_.vel, x_.pos, x_.rot));
  }

  // calculated the pos and attitude prediction at the frame-end
  double sign = cloud_end_time > imu_end_time ? 1.0 : -1.0;
  dt = sign * (cloud_end_time - imu_end_time);
  predict(dt, Q_, u);

  last_imu_ = sensor_data.imu.back();
  last_lidar_end_time_ = cloud_end_time;

  // undistort each lidar point (backward propagation)
  if (cloud.points.begin() == cloud.points.end())
    return;
  auto point = cloud.points.end() - 1;
  for (auto it_kp = imu_pose_.end() - 1; it_kp != imu_pose_.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    Mat3 &R_imu = head->rot;
    Vec3 &vel_imu = head->vel;
    Vec3 &pos_imu = head->pos;
    Vec3 &acc_imu = head->acc;
    Vec3 &gyr = tail->gyr;

    for (; point->time > head->offset_time; point--)
    {
      dt = point->time - head->offset_time;

      // Transform to the 'end' frame, using only the rotation
      Mat3 R_w_curi(R_imu * SO3Expmap(gyr, dt)); // cur to world
      Vec3 t_w_curi(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - x_.pos);

      Vec3 p_l(point->x, point->y, point->z);        // lidar frame point
      Vec3 p_i(x_.Ril * p_l + x_.til); // imu frame point
      Vec3 p_w(R_w_curi * p_i + t_w_curi);           // world frame point

      Mat3 &R_w_endi = x_.rot;

      Vec3 p_deskew = x_.Ril.transpose() * (R_w_endi.transpose() * p_w - x_.til); // using only the rotation

      // save Undistorted points and their rotation
      point->x = p_deskew(0);
      point->y = p_deskew(1);
      point->z = p_deskew(2);

      if (point == cloud.points.begin())
        break;
    }
  }
}

void Esekf::propagation(const SensorData &sensor_data, CloudPtr &cloud_deskew)
{
  if (sensor_data.imu.empty())
  {
    return;
  };

  if (imu_need_init_)
  {
    if (initImu(sensor_data))
    {
      imu_need_init_ = false;
    }
    return;
  }

  ROS_ASSERT(sensor_data.cloud != nullptr);
  undistortCloud(sensor_data, *cloud_deskew);
}

// Forward Propagation  III-C
void Esekf::predict(double &dt, MatNN &Q, const InputU &u)
{
  VecS f = f_func(x_, u, dt);                                  // paper (3) f
  MatSS f_x = df_dx_func(x_, u, dt);                           // paper (7) df/dx
  MatSN f_w = df_dw_func(x_, u, dt);                           // paper (7) df/dw
  x_ = x_.plus(f);                                             // paper (4)
  P_ = f_x * P_ * f_x.transpose() + f_w * Q * f_w.transpose(); // paper (8) MatSS of Forward Propagation
}

// ESKF
void Esekf::iteratedUpdate(CloudPtr &cloud_ds)
{
  HData h_data;
  h_data.converge = true;
  int t = 0;
  State x_propagated = x_; // forward propagated state, paper (18) x^
  MatSS P_propagated = P_;

  for (int i = -1; i < maximum_iter_; i++)
  {
    if (!h_model_(h_data, x_, cloud_ds))
    {
      continue;
    }

    VecS delta_x = x_.minus(x_propagated); // paper (18) x^k - x^

    auto& H1 = h_data.Hessian;
    auto& g1 = h_data.gradient;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;
    //K = (H.transpose() * H * R_inv_ + P_.inverse()).inverse() * H.transpose() * R_inv_; // paper (20)
    //VecS dx = -K * z - (MatSS::Identity() - K * H) * delta_x; // paper (18) notice: J_inv = I

    MatSS Hessian = R_inv_ * H1  + P_.inverse();
    MatSS Hessian_inv = Hessian.inverse();
    VecS  gradient = R_inv_ * g1 + P_.inverse() * delta_x;
    VecS  dx = -Hessian_inv * gradient;
    
    x_ = x_.plus(dx); // update current state. paper (18)

    h_data.converge = true;
    for (int i = 0; i < SZ; i++)
    {
      if (std::fabs(dx[i]) > epsi_)
      {
        h_data.converge = false;
        break;
      }
    }

    if (h_data.converge)
      t++;

    if (!t && i == maximum_iter_ - 2)
    {
      h_data.converge = true;
    }

    if (t > 1 || i == maximum_iter_ - 1)
    {
      //P_ = (MatSS::Identity() - K * H) * P_; // paper (19)
      P_ = Hessian_inv;
      return;
    }
  }
}

MatNN processNoiseCov()
{
  MatNN Q = Eigen::MatrixXd::Zero(NZ, NZ);

  Q.block<3, 3>(L_Nw, L_Nw) = 0.0001 * Eigen::Matrix3d::Identity();
  Q.block<3, 3>(L_Na, L_Na) = 0.0001 * Eigen::Matrix3d::Identity();
  Q.block<3, 3>(L_Nbw, L_Nbw) = 0.00001 * Eigen::Matrix3d::Identity();
  Q.block<3, 3>(L_Nba, L_Nba) = 0.00001 * Eigen::Matrix3d::Identity();
  return Q;
}
