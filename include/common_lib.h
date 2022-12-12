#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fast_lio/Pose6D.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

#define PI_M (3.14159265358)
#define G_m_s2 (9.81)   // Gravaty const in GuangDong/China
#define DIM_STATE (18)  // Dimension of states (Let Dim(SO(3)) = 3)
#define DIM_PROC_N (12) // Dimension of process noise (Let Dim(SO(3)) = 3)
#define LIDAR_SP_LEN (2)
#define INIT_COV (1)
#define NUM_MATCH_POINTS (5)
#define MAX_MEAS_DIM (10000)

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define CONSTRAIN(v, min, max) ((v > min) ? ((v < max) ? v : max) : min)
#define ARRAY_FROM_EIGEN(mat) mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat) vector<decltype(mat)::Scalar>(mat.data(), mat.data() + mat.rows() * mat.cols())

typedef fast_lio::Pose6D Pose6D;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Matrix3d Mat3;
typedef Eigen::Matrix<double, 4, 4> Mat4;


struct SensorData // Lidar data and imu dates for the curent process
{
  SensorData()
  {
    lidar_beg_time = 0.0;
    this->lidar.reset(new PointCloud());
  };
  double lidar_beg_time;
  double lidar_end_time;
  PointCloud::Ptr lidar;
  std::deque<sensor_msgs::Imu::ConstPtr> imu;
};

Pose6D set_pose6d(const double t, const Eigen::Matrix<double, 3, 1> &a, const Eigen::Matrix<double, 3, 1> &g,
                const Eigen::Matrix<double, 3, 1> &v, const Eigen::Matrix<double, 3, 1> &p, const Eigen::Matrix<double, 3, 3> &R);

float calc_dist(PointType p1, PointType p2);

bool esti_plane(Eigen::Matrix<double, 4, 1> &pca_result, const PointVector &point, const double threshold);

#endif