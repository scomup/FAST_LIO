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
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include "preprocess.h"

#include "mapping.h"

Mapping::Mapping(bool extrinsic_est, double filter_size_map)
{
  extrinsic_est_ = extrinsic_est;
  filter_size_map_ = filter_size_map;
}

void Mapping::setState(ESEKF::State &state)
{
  state_ = state;
}

void Mapping::pointL2W(PointType const *const pi, PointType *const po)
{
  Vec3 p_lidar(pi->x, pi->y, pi->z);
  Vec3 p_global(state_.rot * (state_.Ril * p_lidar + state_.til) + state_.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

template <typename T>
void Mapping::pointL2W(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
{
  po = (state_.rot * (state_.Ril * pi + state_.til) + state_.pos);
}

void Mapping::updateMapArea(Vec3 &pos_LiD)
{
  // Removes the point far from the current position.
  return;
}

bool Mapping::initMap(const PointCloud::Ptr &cloud)
{
  /*** initialize the map kdtree ***/
  if (ikdtree_.Root_Node == nullptr)
  {

    int cloud_size = cloud->points.size();
    if (cloud_size > 5)
    {
      PointCloud::Ptr cloud_world(new PointCloud(cloud_size, 1));
      ikdtree_.set_downsample_param(filter_size_map_);
      for (int i = 0; i < cloud_size; i++)
      {
        pointL2W(&(cloud->points[i]), &(cloud_world->points[i]));
      }
      ikdtree_.Build(cloud_world->points);
      return true;
    }
  }
  return false;
}

void Mapping::hModel(ESEKF::HData &h_data, ESEKF::State &state, PointCloud::Ptr &cloud)
{
  int cloud_size = cloud->points.size();
  norms_.resize(cloud_size);
  residuals_.resize(cloud_size);
  neighbor_array_.resize(cloud_size);

  std::vector<int> good_index;

  for (int i = 0; i < cloud_size; i++) // 遍历所有的特征点
  {
    PointType &point = cloud->points[i];
    PointType point_world;

    Vec3 p_l(point.x, point.y, point.z);
    // 把Lidar坐标系的点先转到IMU坐标系，再根据前向传播估计的位姿x，转到世界坐标系
    Vec3 p_global(state.rot * (state.Ril * p_l + state.til) + state.pos);
    point_world.x = p_global(0);
    point_world.y = p_global(1);
    point_world.z = p_global(2);
    point_world.intensity = point.intensity;

    std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
    auto &points_near = neighbor_array_[i]; // neighbor_array_[i]打印出来发现是按照离point_world距离，从小到大的顺序的vector
    if (h_data.converge)
    {
      // 寻找point_world的最近邻的平面点
      ikdtree_.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);

      if(points_near.size() < NUM_MATCH_POINTS)
        continue;
      if(pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5)
        continue;
    }

    Vec4 plane;

    if (esti_plane(plane, points_near, 0.1))
    {
      float r = plane(0) * point_world.x + plane(1) * point_world.y + plane(2) * point_world.z + plane(3); // 当前点到平面的距离
      float s = 1 - 0.9 * fabs(r) / sqrt(p_l.norm());                                                      // 如果残差大于经验阈值，则认为该点是有效点  简言之，距离原点越近的lidar点  要求点到平面的距离越苛刻

      if (s < 0.9) 
        continue;

      good_index.push_back(i);

      norms_[i] = plane.head<3>();
      residuals_[i] = r;
    }
  }

  if (good_index.size() < 1)
  {
    h_data.valid = false;
    ROS_WARN("No Effective Points! \n");
    return;
  }

  h_data.h = Eigen::MatrixXd::Zero(good_index.size(), ESEKF::SZ);
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
      h_data.h.block<1, 3>(idx, ESEKF::L_P) = n;
      h_data.h.block<1, 3>(idx, ESEKF::L_R) = A;
      h_data.h.block<1, 3>(idx, ESEKF::L_Rli) = B;
      h_data.h.block<1, 3>(idx, ESEKF::L_Tli) = C;
    }
    else
    {
      h_data.h.block<1, 3>(idx, ESEKF::L_P) = n;
      h_data.h.block<1, 3>(idx, ESEKF::L_R) = A;
    }

    h_data.z(idx) = residuals_[i];
  }
}

void Mapping::updateMap(PointCloud::Ptr cloud)
{
  PointVector new_points;
  PointVector new_points_ds;
  int cloud_size = cloud->points.size();

  PointCloud::Ptr cloud_world(new PointCloud(cloud_size, 1));

  for (int i = 0; i < cloud_size; i++)
  {
    /* transform to world frame */
    pointL2W(&(cloud->points[i]), &(cloud_world->points[i]));
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
      float dist = calc_dist(cloud_world->points[i], mid_point);
      if (fabs(neighbors[0].x - mid_point.x) > 0.5 * filter_size_map_ && fabs(neighbors[0].y - mid_point.y) > 0.5 * filter_size_map_ && fabs(neighbors[0].z - mid_point.z) > 0.5 * filter_size_map_)
      {
        new_points_ds.push_back(cloud_world->points[i]);
        continue;
      }
      for (int j = 0; j < neighbors.size(); j++)
      {
        if (calc_dist(neighbors[j], mid_point) < dist)
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

void Mapping::pubCloud(const ros::Publisher &pub_cloud, PointCloud::Ptr &cloud, double time)
{
  int size = cloud->points.size();
  PointCloud::Ptr cloud_world(new PointCloud(size, 1));
  for (int i = 0; i < size; i++)
  {
    pointL2W(&cloud->points[i], &cloud_world->points[i]);
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_world, cloud_msg);
  cloud_msg.header.stamp = ros::Time().fromSec(time);
  cloud_msg.header.frame_id = "map";
  pub_cloud.publish(cloud_msg);
}

template <typename T>
void Mapping::setOdomMsg(T &out)
{
  out.pose.position.x = state_.pos(0);
  out.pose.position.y = state_.pos(1);
  out.pose.position.z = state_.pos(2);
  Eigen::Quaterniond q = Eigen::Quaterniond(state_.rot);
  out.pose.orientation.x = q.x();
  out.pose.orientation.y = q.y();
  out.pose.orientation.z = q.z();
  out.pose.orientation.w = q.w();
}

void Mapping::pubOdom(const ros::Publisher &pub, const ESEKF::Esekf &kf, double time)
{
  nav_msgs::Odometry odom;

  odom.header.frame_id = "map";
  odom.child_frame_id = "imu";
  odom.header.stamp = ros::Time().fromSec(time);
  setOdomMsg(odom.pose);
  pub.publish(odom);
  auto P = kf.getP();
  for (int i = 0; i < 6; i++)
  {
    int k = i < 3 ? i + 3 : i - 3;
    odom.pose.covariance[i * 6 + 0] = P(k, 3);
    odom.pose.covariance[i * 6 + 1] = P(k, 4);
    odom.pose.covariance[i * 6 + 2] = P(k, 5);
    odom.pose.covariance[i * 6 + 3] = P(k, 0);
    odom.pose.covariance[i * 6 + 4] = P(k, 1);
    odom.pose.covariance[i * 6 + 5] = P(k, 2);
  }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(odom.pose.pose.position.x,
                                  odom.pose.pose.position.y,
                                  odom.pose.pose.position.z));
  q.setW(odom.pose.pose.orientation.w);
  q.setX(odom.pose.pose.orientation.x);
  q.setY(odom.pose.pose.orientation.y);
  q.setZ(odom.pose.pose.orientation.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "map", "imu"));
}
