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
}

void Mapping::updateMapArea(Vec3 &pos_LiD)
{
  // Removes the point far from the current position.
  return;
}

bool Mapping::initMap(const PointCloud::Ptr &cloud, const State &state)
{
  // initialize the map kdtree 
  if (ikdtree_.Root_Node == nullptr)
  {

    int cloud_size = cloud->points.size();
    if (cloud_size > 5)
    {
      PointCloud::Ptr cloud_world(new PointCloud(cloud_size, 1));
      ikdtree_.set_downsample_param(filter_size_map_);
      for (int i = 0; i < cloud_size; i++)
      {
        pointL2W(&(cloud->points[i]), &(cloud_world->points[i]), state);
      }
      ikdtree_.Build(cloud_world->points);
      return true;
    }
  }
  return false;
}

bool Mapping::point2PlaneModel(HData &h_data, State &state, PointCloud::Ptr &cloud)
{
  int cloud_size = cloud->points.size();
  norms_.resize(cloud_size);
  residuals_.resize(cloud_size);
  neighbor_array_.resize(cloud_size);

  std::vector<int> good_index;

  for (int i = 0; i < cloud_size; i++) 
  {
    PointType &point = cloud->points[i];
    PointType point_world;

    Vec3 pl(point.x, point.y, point.z);
    Vec3 pw(state.rot * (state.Ril * pl + state.til) + state.pos);
    point_world.x = pw(0);
    point_world.y = pw(1);
    point_world.z = pw(2);
    point_world.intensity = point.intensity;

    std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
    auto &points_near = neighbor_array_[i];
    if (h_data.converge)
    {
      ikdtree_.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);

      if(points_near.size() < NUM_MATCH_POINTS)
        continue;
      if(pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5)
        continue;
    }

    Vec4 plane;

    if (calcPlane(plane, points_near, 0.1))
    {
      float r = plane(0) * point_world.x + plane(1) * point_world.y + plane(2) * point_world.z + plane(3); 
      float s = 1 - 0.9 * fabs(r) / sqrt(pl.norm());

      if (s < 0.9) 
        continue;

      good_index.push_back(i);

      norms_[i] = plane.head<3>();
      residuals_[i] = r;
    }
  }

  if (good_index.size() < 1)
  {
    ROS_WARN("No Effective Points! \n");
    return false;
  }

  h_data.h = Eigen::MatrixXd::Zero(good_index.size(), SZ);
  h_data.z.resize(good_index.size());

  for (int idx = 0; idx < good_index.size(); idx++)
  {
    int i = good_index[idx];

    Vec3 point = cloud->points[i].getVector3fMap().template cast<double>(); // point in lidar frame

    Mat3 point_skew  = skewSymMat(point);

    Vec3 point_i = state.Ril * point + state.til;  // point in imu frame
    Mat3 point_i_skew = skewSymMat(point_i);

    Vec3& n = norms_[i];

    Vec3 C(state.rot.transpose() * n);
    Vec3 A(point_i_skew * C);
    if (extrinsic_est_)
    {
      Vec3 B(point_skew * state.Ril.transpose() * C);
      h_data.h.block<1, 3>(idx, L_P) = n;
      h_data.h.block<1, 3>(idx, L_R) = A;
      h_data.h.block<1, 3>(idx, L_Rli) = B;
      h_data.h.block<1, 3>(idx, L_Tli) = C;
    }
    else
    {
      h_data.h.block<1, 3>(idx, L_P) = n;
      h_data.h.block<1, 3>(idx, L_R) = A;
    }

    h_data.z(idx) = residuals_[i];
  }
  return true;
}

void Mapping::updateMap(PointCloud::Ptr cloud, const State &state)
{
  PointVector new_points;
  PointVector new_points_ds;
  int cloud_size = cloud->points.size();

  PointCloud::Ptr cloud_world(new PointCloud(cloud_size, 1));

  for (int i = 0; i < cloud_size; i++)
  {
    /* transform to world frame */
    pointL2W(&(cloud->points[i]), &(cloud_world->points[i]), state);
    /* decide if need add to map */
    if (!neighbor_array_[i].empty())
    {
      const PointVector &neighbors = neighbor_array_[i];
      bool need_add = true;
      BoxPointType Box_of_Point;
      PointType downsample_result, mid_point;
      mid_point.x = floor(cloud_world->points[i].x / filter_size_map_) * filter_size_map_ + 0.5 * filter_size_map_;
      mid_point.y = floor(cloud_world->points[i].y / filter_size_map_) * filter_size_map_ + 0.5 * filter_size_map_;
      mid_point.z = floor(cloud_world->points[i].z / filter_size_map_) * filter_size_map_ + 0.5 * filter_size_map_;
      float dist = pcl::squaredEuclideanDistance(cloud_world->points[i], mid_point);
      if (fabs(neighbors[0].x - mid_point.x) > 0.5 * filter_size_map_ && fabs(neighbors[0].y - mid_point.y) > 0.5 * filter_size_map_ && fabs(neighbors[0].z - mid_point.z) > 0.5 * filter_size_map_)
      {
        new_points_ds.push_back(cloud_world->points[i]);
        continue;
      }
      for (int j = 0; j < neighbors.size(); j++)
      {
        if (pcl::squaredEuclideanDistance(neighbors[j], mid_point) < dist)
        {
          need_add = false;
          break;
        }
      }
      if (need_add)
        new_points.push_back(cloud_world->points[i]);
    }
    else
    {
      new_points.push_back(cloud_world->points[i]);
    }
  }

  ikdtree_.Add_Points(new_points, true);
  ikdtree_.Add_Points(new_points_ds, false);
}

