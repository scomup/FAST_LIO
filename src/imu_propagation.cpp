
#include "imu_propagation.h"


#define MAX_INI_COUNT (10)

const bool time_cmp(PointType &x, PointType &y) { return (x.time < y.time); };

IMUPropagation::IMUPropagation()
    :imu_need_init_(true), start_timestamp_(-1)
{
  init_imu_num_ = 0;
  Q_ = ESEKF::processNoiseCov();
  cov_acc_ = Vec3(0.1, 0.1, 0.1);
  cov_gyr_ = Vec3(0.1, 0.1, 0.1);
  cov_bias_gyr_ = Vec3(0.0001, 0.0001, 0.0001);
  cov_bias_acc_ = Vec3(0.0001, 0.0001, 0.0001);
  gyr_last_ = Vec3::Zero();
  til_ = Vec3::Zero();
  Ril_ = Mat3::Identity();
  last_imu_.reset(new sensor_msgs::Imu());
}

IMUPropagation::~IMUPropagation() {}


void IMUPropagation::setExtrinsic(const Mat4 &T)
{
  til_ = T.block<3, 1>(0, 3);
  Ril_ = T.block<3, 3>(0, 0);
}

void IMUPropagation::setExtrinsic(const Vec3 &transl)
{
  til_ = transl;
  Ril_.setIdentity();
}

void IMUPropagation::setExtrinsic(const Vec3 &transl, const Mat3 &rot)
{
  til_ = transl;
  Ril_ = rot;
}

void IMUPropagation::setGyrCov(const Vec3 &cov_gyr)
{
  cov_gyr_ = cov_gyr;
}

void IMUPropagation::setAccCov(const Vec3 &cov_acc)
{
  cov_acc_ = cov_acc;
}

void IMUPropagation::setGyrBiasCov(const Vec3 &b_g)
{
  cov_bias_gyr_ = b_g;
}

void IMUPropagation::setAccBiasCov(const Vec3 &b_a)
{
  cov_bias_acc_ = b_a;
}

bool IMUPropagation::init(const SensorData &sensor_data, ESEKF::Esekf &kf_state)
{
  //initializing ESEKF state
  for (const auto &imu : sensor_data.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &imu_gyr = imu->angular_velocity;
    mean_acc_ += Vec3(imu_acc.x,imu_acc.y,imu_acc.z);
    mean_gyr_ += Vec3(imu_gyr.x,imu_gyr.y,imu_gyr.z);
    init_imu_num_ ++;
  }

  if (init_imu_num_ > MAX_INI_COUNT)
  {
    mean_acc_ /= init_imu_num_;
    mean_gyr_ /= init_imu_num_;
    ESEKF::State init_state = kf_state.getState();
    init_state.grav = -mean_acc_ / mean_acc_.norm() * Gravity_;
    init_state.bg = mean_gyr_;
    init_state.til = til_;
    init_state.Ril = Eigen::Quaterniond(Ril_);
    kf_state.setState(init_state);
    Eigen::Matrix<double, ESEKF::SZ, ESEKF::SZ> P0 = kf_state.getP();
    P0.setIdentity();
    P0(6, 6) = P0(7, 7) = P0(8, 8) = 0.00001;
    P0(9, 9) = P0(10, 10) = P0(11, 11) = 0.00001;
    P0(15, 15) = P0(16, 16) = P0(17, 17) = 0.0001;
    P0(18, 18) = P0(19, 19) = P0(20, 20) = 0.001;
    P0(21, 21) = P0(22, 22) = 0.00001;
    kf_state.setP(P0);
    last_imu_ = sensor_data.imu.back();
    Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
    Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
    Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
    Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;
    return true;
  }
  return false;
}

void IMUPropagation::undistortCloud(const SensorData &sensor_data, ESEKF::Esekf &kf_state, PointCloud &cloud)
{
  // add the imu of the last frame-tail to the of current frame-head 
  auto imus = sensor_data.imu;
  imus.push_front(last_imu_);
  const double &imu_beg_time = imus.front()->header.stamp.toSec();
  const double &imu_end_time = imus.back()->header.stamp.toSec();
  const double &pcl_beg_time = sensor_data.lidar_beg_time;
  const double &pcl_end_time = sensor_data.lidar_end_time;

  // sort point clouds by offset time 
  cloud = *(sensor_data.lidar);
  sort(cloud.points.begin(), cloud.points.end(), time_cmp);

  // Initialize IMU pose 
  ESEKF::State imu_state = kf_state.getState();
  imu_pose_.clear();
  imu_pose_.push_back(ESEKF::BPInfo(0.0, acc_last_, gyr_last_, imu_state.vel, imu_state.pos, imu_state.rot));


  double dt = 0;

  ESEKF::InputU u;
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
    kf_state.predict(dt, Q_, u);

    /* save the poses at each IMU measurements */
    imu_state = kf_state.getState();
    gyr_last_ = gyr - imu_state.bg;
    acc_last_ = imu_state.rot * (acc - imu_state.ba);

    acc_last_ += imu_state.grav;

    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    imu_pose_.push_back(ESEKF::BPInfo(offs_t, acc_last_, gyr_last_, imu_state.vel, imu_state.pos, imu_state.rot));
  }

  // calculated the pos and attitude prediction at the frame-end 
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q_, u);

  imu_state = kf_state.getState();
  last_imu_ = sensor_data.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  // undistort each lidar point (backward propagation) 
  if (cloud.points.begin() == cloud.points.end())
    return;
  auto point = cloud.points.end() - 1;
  for (auto it_kp = imu_pose_.end() - 1; it_kp != imu_pose_.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    Mat3& R_imu = head->rot;
    Vec3& vel_imu = head->vel;
    Vec3& pos_imu = head->pos;
    Vec3& acc_imu = head->acc;
    Vec3& gyr = tail->gyr;

    for (; point->time > head->offset_time; point--)
    {
      dt = point->time - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * p_deskew = R_imu_e ^ T * (R_i * p_i + t_ei) where t_ei is represented in global frame */
      Mat3 R_i(R_imu * SO3Expmap(gyr, dt));

      Vec3 p_i(point->x, point->y, point->z);
      Vec3 t_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      // imu_state.Ril * p_i + imu_state.til = pi
      //  (R_i * pi + t_ei) = p_ie
      // imu_state.rot = Rwi
      
      Vec3 p_deskew = imu_state.Ril.transpose() * (imu_state.rot.transpose() * (R_i * (imu_state.Ril * p_i + imu_state.til) + t_ei) - imu_state.til); // not accurate!

      // save Undistorted points and their rotation
      point->x = p_deskew(0);
      point->y = p_deskew(1);
      point->z = p_deskew(2);

      if (point == cloud.points.begin())
        break;
    }
  }
}

void IMUPropagation::process(const SensorData &sensor_data, ESEKF::Esekf &kf_state, PointCloud::Ptr &cur_pcl_un)
{
  if (sensor_data.imu.empty())
  {
    return;
  };

  if (imu_need_init_)
  {
    if(init(sensor_data, kf_state))
    {
      imu_need_init_ = false;
    }
    return;
  }

  ROS_ASSERT(sensor_data.lidar != nullptr);
  undistortCloud(sensor_data, kf_state, *cur_pcl_un);
}
