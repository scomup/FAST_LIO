#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include "point_types.h"

#define PI_M (3.14159265358)
#define Gravity_ (9.81)   // Gravaty const in GuangDong/China
#define NUM_MATCH_POINTS (5)

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define CONSTRAIN(v, min, max) ((v > min) ? ((v < max) ? v : max) : min)
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat) vector<decltype(mat)::Scalar>(mat.data(), mat.data() + mat.rows() * mat.cols())

typedef PointXYZIT PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix3d Mat3;
typedef Eigen::Matrix<double, 4, 4> Mat4;

struct LidarData
{
  PointCloud::Ptr cloud;
  double stamp;
};

struct SensorData // Lidar data and imu dates for the curent process
{
  SensorData()
  {
    this->lidar.reset(new PointCloud());
  };
  double lidar_stamp;
  PointCloud::Ptr lidar;
  std::deque<sensor_msgs::Imu::ConstPtr> imu;
};

#endif