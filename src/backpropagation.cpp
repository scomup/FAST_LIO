
#include "backpropagation.h"


#define MAX_INI_COUNT (10)

const bool time_list(PointType &x, PointType &y) { return (x.time < y.time); };

BacKPropagationIMU::BacKPropagationIMU()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q_ = ESEKF::processNoiseCov();
  cov_acc_ = Vec3(0.1, 0.1, 0.1);
  cov_gyr_ = Vec3(0.1, 0.1, 0.1);
  cov_bias_gyr_ = Vec3(0.0001, 0.0001, 0.0001);
  cov_bias_acc_ = Vec3(0.0001, 0.0001, 0.0001);
  mean_acc_ = Vec3(0, 0, -1.0);
  mean_gyr_ = Vec3(0, 0, 0);
  gyr_last_ = Vec3::Zero();
  til_ = Vec3::Zero();
  Ril_ = Mat3::Identity();
  last_imu_.reset(new sensor_msgs::Imu());
}

BacKPropagationIMU::~BacKPropagationIMU() {}

void BacKPropagationIMU::reset()
{
  // ROS_WARN("reset BacKPropagationIMU");
  mean_acc_ = Vec3(0, 0, -1.0);
  mean_gyr_ = Vec3(0, 0, 0);
  gyr_last_ = Vec3::Zero();
  imu_need_init_ = true;
  start_timestamp_ = -1;
  init_iter_num = 1;
  v_imu_.clear();
  imu_pose_.clear();
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloud());
}

void BacKPropagationIMU::setExtrinsic(const Mat4 &T)
{
  til_ = T.block<3, 1>(0, 3);
  Ril_ = T.block<3, 3>(0, 0);
}

void BacKPropagationIMU::setExtrinsic(const Vec3 &transl)
{
  til_ = transl;
  Ril_.setIdentity();
}

void BacKPropagationIMU::setExtrinsic(const Vec3 &transl, const Mat3 &rot)
{
  til_ = transl;
  Ril_ = rot;
}

void BacKPropagationIMU::setGyrCov(const Vec3 &scaler)
{
  cov_gyr_scale_ = scaler;
}

void BacKPropagationIMU::setAccCov(const Vec3 &scaler)
{
  cov_acc_scale_ = scaler;
}

void BacKPropagationIMU::setGyrBiasCov(const Vec3 &b_g)
{
  cov_bias_gyr_ = b_g;
}

void BacKPropagationIMU::setAccBiasCov(const Vec3 &b_a)
{
  cov_bias_acc_ = b_a;
}

void BacKPropagationIMU::init(const SensorData &sensor_data, ESEKF::Esekf &kf_state, int &N)
{
  /** 1. initializing the gravity, gyr bias, acc and gyr covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/

  Vec3 cur_acc, cur_gyr;

  if (b_first_frame_)
  {
    reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = sensor_data.imu.front()->linear_acceleration;
    const auto &gyr_acc = sensor_data.imu.front()->angular_velocity;
    mean_acc_ << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr_ << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time_ = sensor_data.lidar_beg_time;
  }

  for (const auto &imu : sensor_data.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc_ += (cur_acc - mean_acc_) / N;
    mean_gyr_ += (cur_gyr - mean_gyr_) / N;

    cov_acc_ = cov_acc_ * (N - 1.0) / N + (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
    cov_gyr_ = cov_gyr_ * (N - 1.0) / N + (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

    // std::cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc_.norm()<<std::endl;

    N++;
  }
  ESEKF::State init_state = kf_state.getState();
  init_state.grav = -mean_acc_ / mean_acc_.norm() * G_m_s2;

  init_state.bg = mean_gyr_;
  init_state.til = til_;
  init_state.Ril = Eigen::Quaterniond(Ril_);
  kf_state.setState(init_state);

  Eigen::Matrix<double, ESEKF::SZ, ESEKF::SZ> init_P = kf_state.getP();
  init_P.setIdentity();
  init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
  init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
  init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
  init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
  init_P(21, 21) = init_P(22, 22) = 0.00001;
  kf_state.setP(init_P);
  last_imu_ = sensor_data.imu.back();
}

void BacKPropagationIMU::undistortCloud(const SensorData &sensor_data, ESEKF::Esekf &kf_state, PointCloud &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = sensor_data.imu;
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double &pcl_beg_time = sensor_data.lidar_beg_time;
  const double &pcl_end_time = sensor_data.lidar_end_time;

  /*** sort point clouds by offset time ***/
  pcl_out = *(sensor_data.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

  /*** Initialize IMU pose ***/
  ESEKF::State imu_state = kf_state.getState();
  imu_pose_.clear();
  imu_pose_.push_back(ESEKF::StateBP(0.0, acc_last_, gyr_last_, imu_state.vel, imu_state.pos, imu_state.rot));


  double dt = 0;

  ESEKF::InputU u;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    if (tail->header.stamp.toSec() < last_lidar_end_time_)
      continue;

    Vec3 gyr_avr(0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z));
    Vec3 acc_avr(0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z));

    acc_avr = acc_avr * G_m_s2 / mean_acc_.norm(); // - state_inout.ba;

    if (head->header.stamp.toSec() < last_lidar_end_time_)
    {
      dt = tail->header.stamp.toSec() - last_lidar_end_time_;
    }
    else
    {
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }

    u.acc = acc_avr;
    u.gyr = gyr_avr;
    Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
    Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
    Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
    Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;
    kf_state.predict(dt, Q_, u);

    /* save the poses at each IMU measurements */
    imu_state = kf_state.getState();
    gyr_last_ = gyr_avr - imu_state.bg;
    acc_last_ = imu_state.rot * (acc_avr - imu_state.ba);

    acc_last_ += imu_state.grav;

    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    imu_pose_.push_back(ESEKF::StateBP(offs_t, acc_last_, gyr_last_, imu_state.vel, imu_state.pos, imu_state.rot));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q_, u);

  imu_state = kf_state.getState();
  last_imu_ = sensor_data.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  /*** undistort each lidar point (backward propagation) ***/
  if (pcl_out.points.begin() == pcl_out.points.end())
    return;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = imu_pose_.end() - 1; it_kp != imu_pose_.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    Mat3& R_imu = head->rot;
    Vec3& vel_imu = head->vel;
    Vec3& pos_imu = head->pos;
    Vec3& acc_imu = head->acc;
    Vec3& gyr_avr = tail->gyr;

    for (; it_pcl->time / double(1000) > head->offset_time; it_pcl--)
    {
      dt = it_pcl->time / double(1000) - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      Mat3 R_i(R_imu * SO3Expmap(gyr_avr, dt));

      Vec3 P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      Vec3 T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      // imu_state.Ril * P_i + imu_state.til = pi
      //  (R_i * pi + T_ei) = p_ie
      // imu_state.rot = Rwi
      Vec3 P_compensate = imu_state.Ril.transpose() * (imu_state.rot.transpose() * (R_i * (imu_state.Ril * P_i + imu_state.til) + T_ei) - imu_state.til); // not accurate!

      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin())
        break;
    }
  }
}

void BacKPropagationIMU::process(const SensorData &sensor_data, ESEKF::Esekf &kf_state, PointCloud::Ptr &cur_pcl_un)
{
  if (sensor_data.imu.empty())
  {
    return;
  };
  ROS_ASSERT(sensor_data.lidar != nullptr);

  if (imu_need_init_)
  {
    /// The very first lidar frame
    init(sensor_data, kf_state, init_iter_num);

    imu_need_init_ = true;

    last_imu_ = sensor_data.imu.back();

    ESEKF::State imu_state = kf_state.getState();
    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc_ *= pow(G_m_s2 / mean_acc_.norm(), 2);
      imu_need_init_ = false;

      cov_acc_ = cov_acc_scale_;
      cov_gyr_ = cov_gyr_scale_;
      ROS_INFO("IMU Initial Done");
    }
    return;
  }

  undistortCloud(sensor_data, kf_state, *cur_pcl_un);
}
