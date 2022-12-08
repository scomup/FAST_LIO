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
#include "ikd_Tree.h"

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

std::mutex mtx_buffer;
std::condition_variable sig_buffer;

std::string root_dir = ROOT_DIR;
std::string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

std::vector<std::vector<int>> pointSearchInd_surf;
std::vector<BoxPointType> cub_needrm;
std::vector<PointVector> Nearest_Points;
std::vector<double> extrinT(3, 0.0);
std::vector<double> extrinR(9, 0.0);
std::deque<double> time_buffer;
std::deque<PointCloud::Ptr> lidar_buffer;
std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloud::Ptr featsFromMap(new PointCloud());
PointCloud::Ptr feats_undistort(new PointCloud());
PointCloud::Ptr feats_down_body(new PointCloud());
PointCloud::Ptr feats_down_world(new PointCloud());
PointCloud::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

Vec3 XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
Vec3 XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
Vec3 position_last(Zero3d);
Vec3 Lidar_T_wrt_IMU(Zero3d);
Mat3 Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekf kf;
State state_point;
Eigen::Vector3d pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

std::shared_ptr<Preprocess> p_pre(new Preprocess());
std::shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
  flg_exit = true;
  ROS_WARN("catch sig %d", sig);
  sig_buffer.notify_all();
}

void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
  Vec3 p_body(pi->x, pi->y, pi->z);
  Vec3 p_global(state_point.rot * (state_point.Rli * p_body + state_point.tli) + state_point.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
{
  Vec3 p_body(pi[0], pi[1], pi[2]);
  Vec3 p_global(state_point.rot * (state_point.Rli * p_body + state_point.tli) + state_point.pos);

  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  Vec3 p_body(pi->x, pi->y, pi->z);
  Vec3 p_global(state_point.rot * (state_point.Rli * p_body + state_point.tli) + state_point.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  Vec3 p_body_lidar(pi->x, pi->y, pi->z);
  Vec3 p_body_imu(state_point.Rli * p_body_lidar + state_point.tli);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

void points_cache_collect()
{
  PointVector points_history;
  ikdtree.acquire_removed_points(points_history);
  // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
  cub_needrm.clear();
  kdtree_delete_counter = 0;
  kdtree_delete_time = 0.0;
  pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
  Vec3 pos_LiD = pos_lid;
  if (!Localmap_Initialized)
  {
    for (int i = 0; i < 3; i++)
    {
      LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
      LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
    }
    Localmap_Initialized = true;
    return;
  }
  float dist_to_map_edge[3][2];
  bool need_move = false;
  for (int i = 0; i < 3; i++)
  {
    dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
    dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
      need_move = true;
  }
  if (!need_move)
    return;
  BoxPointType New_LocalMap_Points, tmp_boxpoints;
  New_LocalMap_Points = LocalMap_Points;
  float mov_dist = std::max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
  for (int i = 0; i < 3; i++)
  {
    tmp_boxpoints = LocalMap_Points;
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
    {
      New_LocalMap_Points.vertex_max[i] -= mov_dist;
      New_LocalMap_Points.vertex_min[i] -= mov_dist;
      tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    }
    else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
    {
      New_LocalMap_Points.vertex_max[i] += mov_dist;
      New_LocalMap_Points.vertex_min[i] += mov_dist;
      tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    }
  }
  LocalMap_Points = New_LocalMap_Points;

  points_cache_collect();
  double delete_begin = omp_get_wtime();
  if (cub_needrm.size() > 0)
    kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
  kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  mtx_buffer.lock();
  scan_count++;
  double preprocess_start_time = omp_get_wtime();
  if (msg->header.stamp.toSec() < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buffer.clear();
  }

  PointCloud::Ptr ptr(new PointCloud());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(msg->header.stamp.toSec());
  last_timestamp_lidar = msg->header.stamp.toSec();
  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  publish_count++;
  // std::cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<std::endl;
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

  if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
  {
    msg->header.stamp =
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
  }

  msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);

  double timestamp = msg->header.stamp.toSec();

  mtx_buffer.lock();

  if (timestamp < last_timestamp_imu)
  {
    ROS_WARN("imu loop back, clear buffer");
    imu_buffer.clear();
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
  if (lidar_buffer.empty() || imu_buffer.empty())
  {
    return false;
  }

  /*** push a lidar scan ***/
  if (!lidar_pushed)
  {
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();
    if (meas.lidar->points.size() <= 1) // time too little
    {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
      ROS_WARN("Too few input point cloud!\n");
    }
    else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
    {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
    }
    else
    {
      scan_num++;
      lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
      lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
    }

    meas.lidar_end_time = lidar_end_time;

    lidar_pushed = true;
  }

  if (last_timestamp_imu < lidar_end_time)
  {
    return false;
  }

  /*** push imu data, and pop from imu buffer ***/
  double imu_time = imu_buffer.front()->header.stamp.toSec();
  meas.imu.clear();
  while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
  {
    imu_time = imu_buffer.front()->header.stamp.toSec();
    if (imu_time > lidar_end_time)
      break;
    meas.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }

  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed = false;
  return true;
}

int process_increments = 0;
void map_incremental()
{
  PointVector PointToAdd;
  PointVector PointNoNeedDownsample;
  PointToAdd.reserve(feats_down_size);
  PointNoNeedDownsample.reserve(feats_down_size);
  for (int i = 0; i < feats_down_size; i++)
  {
    /* transform to world frame */
    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
    /* decide if need add to map */
    if (!Nearest_Points[i].empty() && flg_EKF_inited)
    {
      const PointVector &points_near = Nearest_Points[i];
      bool need_add = true;
      BoxPointType Box_of_Point;
      PointType downsample_result, mid_point;
      mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
      mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
      mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
      float dist = calc_dist(feats_down_world->points[i], mid_point);
      if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
      {
        PointNoNeedDownsample.push_back(feats_down_world->points[i]);
        continue;
      }
      for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
      {
        if (points_near.size() < NUM_MATCH_POINTS)
          break;
        if (calc_dist(points_near[readd_i], mid_point) < dist)
        {
          need_add = false;
          break;
        }
      }
      if (need_add)
        PointToAdd.push_back(feats_down_world->points[i]);
    }
    else
    {
      PointToAdd.push_back(feats_down_world->points[i]);
    }
  }

  double st_time = omp_get_wtime();
  add_point_size = ikdtree.Add_Points(PointToAdd, true);
  ikdtree.Add_Points(PointNoNeedDownsample, false);
  add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
  kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloud::Ptr pcl_wait_pub(new PointCloud(500000, 1));
PointCloud::Ptr pcl_wait_save(new PointCloud());
void publish_frame_world(const ros::Publisher &pubLaserCloudFull)
{
  if (scan_pub_en)
  {
    PointCloud::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloud::Ptr laserCloudWorld(
        new PointCloud(size, 1));

    for (int i = 0; i < size; i++)
    {
      RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                          &laserCloudWorld->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFull.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (pcd_save_en)
  {
    int size = feats_undistort->points.size();
    PointCloud::Ptr laserCloudWorld(
        new PointCloud(size, 1));

    for (int i = 0; i < size; i++)
    {
      RGBpointBodyToWorld(&feats_undistort->points[i],
                          &laserCloudWorld->points[i]);
    }
    *pcl_wait_save += *laserCloudWorld;

    static int scan_wait_num = 0;
    scan_wait_num++;
    if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      pcd_index++;
      std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/scans_") + boost::to_string(pcd_index) + std::string(".pcd"));
      pcl::PCDWriter pcd_writer;
      std::cout << "current scan saved to /PCD/" << all_points_dir << std::endl;
      pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
      pcl_wait_save->clear();
      scan_wait_num = 0;
    }
  }
}

void publish_frame_body(const ros::Publisher &pubLaserCloudFull_body)
{
  int size = feats_undistort->points.size();
  PointCloud::Ptr laserCloudIMUBody(new PointCloud(size, 1));

  for (int i = 0; i < size; i++)
  {
    RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                           &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
  laserCloudmsg.header.frame_id = "body";
  pubLaserCloudFull_body.publish(laserCloudmsg);
  publish_count -= PUBFRAME_PERIOD;
}

void publish_map(const ros::Publisher &pubLaserCloudMap)
{
  sensor_msgs::PointCloud2 laserCloudMap;
  pcl::toROSMsg(*featsFromMap, laserCloudMap);
  laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
  laserCloudMap.header.frame_id = "camera_init";
  pubLaserCloudMap.publish(laserCloudMap);
}

template <typename T>
void set_posestamp(T &out)
{
  out.pose.position.x = state_point.pos(0);
  out.pose.position.y = state_point.pos(1);
  out.pose.position.z = state_point.pos(2);
  out.pose.orientation.x = geoQuat.x;
  out.pose.orientation.y = geoQuat.y;
  out.pose.orientation.z = geoQuat.z;
  out.pose.orientation.w = geoQuat.w;
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "body";
  odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
  set_posestamp(odomAftMapped.pose);
  pubOdomAftMapped.publish(odomAftMapped);
  auto P = kf.get_P();
  for (int i = 0; i < 6; i++)
  {
    int k = i < 3 ? i + 3 : i - 3;
    odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
    odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
    odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
    odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
    odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
    odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
  }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                  odomAftMapped.pose.pose.position.y,
                                  odomAftMapped.pose.pose.position.z));
  q.setW(odomAftMapped.pose.pose.orientation.w);
  q.setX(odomAftMapped.pose.pose.orientation.x);
  q.setY(odomAftMapped.pose.pose.orientation.y);
  q.setZ(odomAftMapped.pose.pose.orientation.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));
}

void publish_path(const ros::Publisher pubPath)
{
  set_posestamp(msg_body_pose);
  msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
  msg_body_pose.header.frame_id = "camera_init";

  /*** if path is too large, the rvis will crash ***/
  static int jjj = 0;
  jjj++;
  if (jjj % 10 == 0)
  {
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;

  nh.param<bool>("publish/path_en", path_en, true);
  nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
  nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);
  nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
  nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
  nh.param<std::string>("map_file_path", map_file_path, "");
  nh.param<std::string>("common/lid_topic", lid_topic, "/velodyne_points");
  nh.param<std::string>("common/imu_topic", imu_topic, "/imu/data");
  nh.param<bool>("common/time_sync_en", time_sync_en, false);
  nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
  nh.param<double>("filter_size_corner", filter_size_corner_min, 0.5);
  nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
  nh.param<double>("filter_size_map", filter_size_map_min, 0.5);
  nh.param<double>("cube_side_length", cube_len, 200);
  nh.param<float>("mapping/det_range", DET_RANGE, 300.f);
  nh.param<double>("mapping/fov_degree", fov_deg, 180);
  nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
  nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
  nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
  nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
  nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
  nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
  nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
  nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
  nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
  nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT, std::vector<double>());
  nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR, std::vector<double>());
  std::cout << "p_pre->lidar_type " << p_pre->lidar_type << std::endl;

  path.header.stamp = ros::Time::now();
  path.header.frame_id = "camera_init";

  /*** variables definition ***/
  int effect_feat_num = 0, frame_num = 0;
  double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
  bool flg_EKF_converged, EKF_stop_flg = 0;

  FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
  HALF_FOV_COS = cos((FOV_DEG)*0.5 * PI_M / 180.0);

  _featsArray.reset(new PointCloud());

  memset(point_selected_surf, true, sizeof(point_selected_surf));
  memset(res_last, -1000.0f, sizeof(res_last));
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
  memset(point_selected_surf, true, sizeof(point_selected_surf));
  memset(res_last, -1000.0f, sizeof(res_last));

  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
  p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
  p_imu->set_gyr_cov(Vec3(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov(Vec3(acc_cov, acc_cov, acc_cov));
  p_imu->set_gyr_bias_cov(Vec3(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu->set_acc_bias_cov(Vec3(b_acc_cov, b_acc_cov, b_acc_cov));

  double epsi[23] = {0.001};
  std::fill(epsi, epsi + 23, 0.001);

  /*** debug record ***/
  FILE *fp;
  std::string pos_log_dir = root_dir + "/Log/pos_log.txt";
  fp = fopen(pos_log_dir.c_str(), "w");

  std::ofstream fout_pre, fout_out, fout_dbg;
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
  fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), std::ios::out);
  if (fout_pre && fout_out)
    std::cout << "~~~~" << ROOT_DIR << " file opened" << std::endl;
  else
    std::cout << "~~~~" << ROOT_DIR << " doesn't exist" << std::endl;

  /*** ROS subscribe initialization ***/
  ros::Subscriber sub_pcl = nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
  ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
  ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
  ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
  ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
  ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
  ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
  ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 100000);
  //------------------------------------------------------------------------------------------------------
  signal(SIGINT, SigHandle);
  ros::Rate rate(5000);
  bool status = ros::ok();
  while (status)
  {
    if (flg_exit)
      break;
    ros::spinOnce();
    if (sync_packages(Measures))
    {
      if (flg_first_scan)
      {
        first_lidar_time = Measures.lidar_beg_time;
        p_imu->first_lidar_time = first_lidar_time;
        flg_first_scan = false;
        continue;
      }

      double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

      match_time = 0;
      kdtree_search_time = 0.0;
      solve_time = 0;
      solve_const_H_time = 0;
      svd_time = 0;
      t0 = omp_get_wtime();

      p_imu->Process(Measures, kf, feats_undistort); // deskew lidar points. by backward propagation

      state_point = kf.get_x();
      pos_lid = state_point.pos + state_point.rot * state_point.tli; // Lidar point in global frame.

      if (feats_undistort->empty() || (feats_undistort == NULL))
      {
        ROS_WARN("No point, skip this scan!\n");
        continue;
      }

      flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
      /*** Segment the map in lidar FOV ***/
      lasermap_fov_segment();

      /*** downsample the feature points in a scan ***/
      downSizeFilterSurf.setInputCloud(feats_undistort);
      downSizeFilterSurf.filter(*feats_down_body);
      t1 = omp_get_wtime();
      feats_down_size = feats_down_body->points.size();
      /*** initialize the map kdtree ***/
      if (ikdtree.Root_Node == nullptr)
      {
        if (feats_down_size > 5)
        {
          ikdtree.set_downsample_param(filter_size_map_min);
          feats_down_world->resize(feats_down_size);
          for (int i = 0; i < feats_down_size; i++)
          {
            pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
          }
          ikdtree.Build(feats_down_world->points);
        }
        continue;
      }
      int featsFromMapNum = ikdtree.validnum();
      kdtree_size_st = ikdtree.size();

      // std::cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<std::endl;

      /*** ICP and iterated Kalman filter update ***/
      if (feats_down_size < 5)
      {
        ROS_WARN("No point, skip this scan!\n");
        continue;
      }

      feats_down_world->resize(feats_down_size);

      pointSearchInd_surf.resize(feats_down_size);
      Nearest_Points.resize(feats_down_size);
      int rematch_num = 0;
      bool nearest_search_en = true; //

      t2 = omp_get_wtime();

      /*** iterated state estimation ***/
      double t_update_start = omp_get_wtime();
      double solve_H_time = 0;
      // kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
      kf.update_iterated_dyn_share_modified(LASER_POINT_COV, feats_down_body, ikdtree, Nearest_Points, NUM_MAX_ITERATIONS, extrinsic_est_en);

      state_point = kf.get_x();
      pos_lid = state_point.pos + state_point.rot * state_point.tli;
      Eigen::Quaterniond q = Eigen::Quaterniond(state_point.rot);
      geoQuat.x = q.x();
      geoQuat.y = q.y();
      geoQuat.z = q.z();
      geoQuat.w = q.w();

      double t_update_end = omp_get_wtime();

      /******* Publish odometry *******/
      publish_odometry(pubOdomAftMapped);

      /*** add the feature points to map kdtree ***/
      t3 = omp_get_wtime();
      map_incremental();
      t5 = omp_get_wtime();

      /******* Publish points *******/
      if (path_en)
        publish_path(pubPath);
      if (scan_pub_en || pcd_save_en)
        publish_frame_world(pubLaserCloudFull);
      if (scan_pub_en && scan_body_pub_en)
        publish_frame_body(pubLaserCloudFull_body);
    }

    status = ros::ok();
    rate.sleep();
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. pcd save will largely influence the real-time performences **/
  if (pcl_wait_save->size() > 0 && pcd_save_en)
  {
    std::string file_name = std::string("scans.pcd");
    std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/") + file_name);
    pcl::PCDWriter pcd_writer;
    std::cout << "current scan saved to /PCD/" << file_name << std::endl;
    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
  }

  fout_out.close();
  fout_pre.close();

  return 0;
}
