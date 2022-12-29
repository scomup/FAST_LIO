#pragma once

#include <deque>
#include <Eigen/Eigen>
#include <sensor_msgs/Imu.h>
#include "point_types.h"

constexpr double GRAVITY = 9.81; 
constexpr int NUM_MATCH_POINTS = 5; 
constexpr int INIT_IMU_NUM = 10; 

// the location of state
constexpr int SZ = 24; // state size
constexpr int NZ = 12; // noise size
// the location of parameters in state vector
constexpr int L_P = 0;
constexpr int L_R = 3;
constexpr int L_Rli = 6;
constexpr int L_Tli = 9;
constexpr int L_V = 12;
constexpr int L_Bw = 15;
constexpr int L_Ba = 18;
constexpr int L_G = 21;
// the location of parameters in noise vector
constexpr int L_Nw = 0;
constexpr int L_Na = 3;
constexpr int L_Nbw = 6;
constexpr int L_Nba = 9;

using MatSS = Eigen::Matrix<double, SZ, SZ>; // 24X24 Cov Mat
using MatSN = Eigen::Matrix<double, SZ, NZ>; // 24X12 Cov Mat
using MatNN = Eigen::Matrix<double, NZ, NZ>; // 12X12 Cov Mat
using VecS = Eigen::Matrix<double, SZ, 1>;   // 24X1 Vec
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Matrix<double, 4, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix<double, 4, 4>;

using PointType = PointXYZIT;
using Cloud = pcl::PointCloud<PointType>;
using CloudPtr = Cloud::Ptr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

struct LidarData
{
  CloudPtr cloud;
  double stamp;
};

struct SensorData
{
  SensorData()
  {
    this->cloud.reset(new Cloud());
  };
  double stamp;
  CloudPtr cloud;
  std::deque<sensor_msgs::Imu::ConstPtr> imu;
};

struct HData
{
  bool converge;
  MatSS Hessian;
  VecS gradient;
};

// Input u (IMU)
struct InputU
{
  Vec3 acc = Vec3(0, 0, 0);
  Vec3 gyr = Vec3(0, 0, 0);
};

struct IMUPose
{
  double offset_time;
  Vec3 acc = Vec3(0, 0, 0);
  Vec3 gyr = Vec3(0, 0, 0);
  Vec3 vel = Vec3(0, 0, 0);
  Vec3 pos = Vec3(0, 0, 0);    // imu postion in world frame
  Mat3 rot = Mat3::Identity(); // imu rotation in world frame

  IMUPose(const double t,
         const Vec3 &a,
         const Vec3 &g,
         const Vec3 &v,
         const Vec3 &p,
         const Mat3 &R)
  {
    offset_time = t;
    acc = a;
    gyr = g;
    vel = v;
    pos = p;
    rot = R;
  }
};