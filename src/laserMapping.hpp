// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
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

#define MAXN (720000)
#define PUBFRAME_PERIOD (20)


std::mutex mtx_;

double last_timestamp_lidar_ = 0, last_timestamp_imu_ = -1.0;
double filter_size_map_ = 0;
double lidar_end_time_ = 0;
bool lidar_pushed_, ekf_inited_ = false;

std::vector<PointVector> neighbor_array_;
std::deque<double> time_buffer_;
std::deque<PointCloud::Ptr> lidar_buffer_;
std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;


KD_TREE<PointType> ikdtree_;

ESEKF::State state_;

std::shared_ptr<Preprocess> p_pre_(new Preprocess());


void pointL2W(PointType const *const pi, PointType *const po)
{
  Vec3 p_lidar(pi->x, pi->y, pi->z);
  Vec3 p_global(state_.rot * (state_.Ril * p_lidar + state_.til) + state_.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

template <typename T>
void pointL2W(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
{
  Vec3 p_body(pi[0], pi[1], pi[2]);
  Vec3 p_global(state_.rot * (state_.Ril * p_body + state_.til) + state_.pos);

  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

void lasermap_fov_segment(Vec3& pos_LiD)
{
  //Removes the point far from the current position.
  return;
}

void cloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  mtx_.lock();
  if (msg->header.stamp.toSec() < last_timestamp_lidar_)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buffer_.clear();
  }

  PointCloud::Ptr ptr(new PointCloud());
  p_pre_->process(msg, ptr);
  lidar_buffer_.push_back(ptr);
  time_buffer_.push_back(msg->header.stamp.toSec());
  last_timestamp_lidar_ = msg->header.stamp.toSec();
  mtx_.unlock();
}

void imuCB(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

  double timestamp = msg->header.stamp.toSec();

  mtx_.lock();

  if (timestamp < last_timestamp_imu_)
  {
    ROS_WARN("imu loop back, clear buffer");
    imu_buffer_.clear();
  }

  last_timestamp_imu_ = timestamp;

  imu_buffer_.push_back(msg);
  mtx_.unlock();
}

double lidar_mean_scantime = 0.0;
int scan_num = 0;
bool syncData(SensorData &sensor_data)
{
  if (lidar_buffer_.empty() || imu_buffer_.empty())
  {
    return false;
  }

  /*** push a lidar scan ***/
  if (!lidar_pushed_)
  {
    sensor_data.lidar = lidar_buffer_.front();
    sensor_data.lidar_beg_time = time_buffer_.front();
    if (sensor_data.lidar->points.size() <= 1) // time too little
    {
      lidar_end_time_ = sensor_data.lidar_beg_time + lidar_mean_scantime;
      ROS_WARN("Too few input point cloud!\n");
    }
    else if (sensor_data.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
    {
      lidar_end_time_ = sensor_data.lidar_beg_time + lidar_mean_scantime;
    }
    else
    {
      scan_num++;
      lidar_end_time_ = sensor_data.lidar_beg_time + sensor_data.lidar->points.back().curvature / double(1000);
      lidar_mean_scantime += (sensor_data.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
    }

    sensor_data.lidar_end_time_ = lidar_end_time_;

    lidar_pushed_ = true;
  }

  if (last_timestamp_imu_ < lidar_end_time_)
  {
    return false;
  }

  /*** push imu data, and pop from imu buffer ***/
  double imu_time = imu_buffer_.front()->header.stamp.toSec();
  sensor_data.imu.clear();
  while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_))
  {
    imu_time = imu_buffer_.front()->header.stamp.toSec();
    if (imu_time > lidar_end_time_)
      break;
    sensor_data.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  lidar_buffer_.pop_front();
  time_buffer_.pop_front();
  lidar_pushed_ = false;
  return true;
}

bool initMap(const PointCloud::Ptr& cloud)
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

void updateMap(PointCloud::Ptr cloud)
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
    if (!neighbor_array_[i].empty() && ekf_inited_)
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

void pubCloud(const ros::Publisher &pub_cloud, PointCloud::Ptr& cloud)
{
  int size = cloud->points.size();
  PointCloud::Ptr cloud_world(new PointCloud(size, 1));
  for (int i = 0; i < size; i++)
  {
    pointL2W(&cloud->points[i],  &cloud_world->points[i]);
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_world, cloud_msg);
  cloud_msg.header.stamp = ros::Time().fromSec(lidar_end_time_);
  cloud_msg.header.frame_id = "map";
  pub_cloud.publish(cloud_msg);

}


template <typename T>
void set_posestamp(T &out)
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

void publish_odometry(const ros::Publisher &pub, const ESEKF::esekf& kf)
{
  nav_msgs::Odometry odom;

  odom.header.frame_id = "map";
  odom.child_frame_id = "imu";
  odom.header.stamp = ros::Time().fromSec(lidar_end_time_); // ros::Time().fromSec(lidar_end_time_);
  set_posestamp(odom.pose);
  pub.publish(odom);
  auto P = kf.get_P();
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

