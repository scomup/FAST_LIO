#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>

#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/distances.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include "preprocess.h"

#include "mapping.h"


bool calcPlane(Eigen::Matrix<double, 4, 1> &pca_result, const PointVector &point, const double threshold)
{
  Eigen::Matrix<double, NUM_MATCH_POINTS, 3> A;
  Eigen::Matrix<double, NUM_MATCH_POINTS, 1> b;
  A.setZero();
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < NUM_MATCH_POINTS; j++)
  {
    A(j, 0) = point[j].x;
    A(j, 1) = point[j].y;
    A(j, 2) = point[j].z;
  }

  Eigen::Matrix<double, 3, 1> norm_vec = A.colPivHouseholderQr().solve(b);

  double n = norm_vec.norm();
  pca_result(0) = norm_vec(0) / n;
  pca_result(1) = norm_vec(1) / n;
  pca_result(2) = norm_vec(2) / n;
  pca_result(3) = 1.0 / n;

  for (int j = 0; j < NUM_MATCH_POINTS; j++)
  {
    if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
    {
      return false;
    }
  }
  return true;
}

void pointL2W(PointType const *const pi, PointType *const po, const State &state)
{
  Vec3 p_lidar(pi->x, pi->y, pi->z);
  Vec3 pw(state.rot * (state.Ril * p_lidar + state.til) + state.pos);

  po->x = pw(0);
  po->y = pw(1);
  po->z = pw(2);
  po->intensity = pi->intensity;
}

template <typename T>
void pointL2W(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po,  const State &state)
{
  po = (state.rot * (state.Ril * pi + state.til) + state.pos);
}

Mapping::Mapping(bool extrinsic_est, double filter_size_map)
{
  extrinsic_est_ = extrinsic_est;
  filter_size_map_ = filter_size_map;
  grid_ = boost::make_shared<NdtGrid<PointType>>();
  grid_->setResolution(2);
}

void Mapping::updateMapArea(Vec3 &pose_lidar)
{
  // Removes the point far from the current position.
  return;
}

bool Mapping::initMap(const PointCloud::Ptr &cloud, const State &state)
{
  if(init_)
  return false;
  int cloud_size = cloud->points.size();
  if (cloud_size > 5)
  {

    PointCloud::Ptr cloud_world(new PointCloud(cloud_size, 1));
    for (int i = 0; i < cloud_size; i++)
    {
      pointL2W(&(cloud->points[i]), &(cloud_world->points[i]), state);
    }
    grid_->setInput(cloud_world);
    init_ = true;
    return true;
  }
  return false;
}

bool Mapping::point2PlaneModel(HData &h_data, State &state, PointCloud::Ptr &cloud)
{
  int cloud_size = cloud->points.size();
  norms_.resize(cloud_size);
  residuals_.resize(cloud_size);
  neighbor_array_.resize(cloud_size);

  std::vector<Eigen::Matrix<double, 3, SZ>> H;
  std::vector<Eigen::Matrix<double, 3, 1>>  Z;

  std::vector<int> good_index;
  for (int i = 0; i < cloud_size; i++) 
  {
    PointType &point = cloud->points[i];
    PointType point_world;

    Vec3 pl(point.x, point.y, point.z);
    Vec3 pi((state.Ril * pl + state.til));
    Vec3 pw(state.rot * pi + state.pos);
    point_world.x = pw.x();
    point_world.y = pw.y();
    point_world.z = pw.z();
    int vid = grid_->getNearest(point_world);

    if(vid == -1)
      continue;

      auto cell = grid_->getCell(vid);
      const Eigen::Vector3d p_mean = cell->mean_;
       Eigen::Matrix3d cinvL = cell->icovL_;
      const Eigen::Vector3d p_diff = pw - p_mean;
      Mat3 pi_skew = skewSymMat(pi);
      Eigen::Matrix<double, 3, SZ> h = Eigen::Matrix<double, 3, SZ>::Zero();
      Eigen::Matrix<double, 3, 1> z = Eigen::Matrix<double, 3, 1>::Zero();
      h.block<3, 3>(0, L_P) = cinvL;
      h.block<3, 3>(0, L_R) = -cinvL * state.rot * pi_skew;
      z = cinvL * p_diff;
      H.push_back(h);
      Z.push_back(z);
    
    
  }


  int n = H.size();
  if (n < 1)
  {
    ROS_WARN("No Effective Points! \n");
    return false;
  }
 
  h_data.h = Eigen::MatrixXd::Zero(3 * n, SZ);
  h_data.z.resize(3 * n);

  for (int idx = 0; idx < n; idx++)
  {
    h_data.h.block<3, SZ>(idx * 3, 0) = H[idx];
    h_data.z.block<3, 1>(idx * 3, 0) = Z[idx];
  }
  
  return true;
}

void Mapping::updateMap(PointCloud::Ptr cloud, const State &state)
{
  int cloud_size = cloud->points.size();
  PointCloud::Ptr cloud_world(new PointCloud(cloud_size, 1));
  for (int i = 0; i < cloud_size; i++)
  {
    pointL2W(&(cloud->points[i]), &(cloud_world->points[i]), state);
  }
  grid_->update(cloud_world);
}
