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

std::shared_ptr<Preprocess> p_pre_(new Preprocess());

class Mapping
{
public:
  double filter_size_map_ = 0;

  bool extrinsic_est_ = true;

  std::vector<PointVector> neighbor_array_;

  KD_TREE<PointType> ikdtree_;

  ESEKF::State state_;

  PointCloud::Ptr normvec;       // 特征点在地图中对应的平面参数(平面的单位法向量,以及当前点到平面距离)
  PointCloud::Ptr laserCloudOri; // 有效特征点
  PointCloud::Ptr corr_normvect; // 有效特征点对应点法相量
  bool point_selected_surf[100000] = {1};                   // 判断是否是有效特征点

  Mapping()
  {
    normvec.reset(new PointCloud());  
    laserCloudOri.reset(new PointCloud(100000, 1));
    corr_normvect.reset(new PointCloud(100000, 1));
  }

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
    po = (state_.rot * (state_.Ril * pi + state_.til) + state_.pos);
  }

  void updateMapArea(Vec3 &pos_LiD)
  {
    // Removes the point far from the current position.
    return;
  }

  bool initMap(const PointCloud::Ptr &cloud)
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

  void h_model(ESEKF::HData &ekfom_data, ESEKF::State& state, PointCloud::Ptr &cloud)
  {
    normvec->clear();
    normvec->resize(int(cloud->points.size()));

    int cloud_size = cloud->points.size();
    laserCloudOri->clear();
    corr_normvect->clear();

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
      if (ekfom_data.converge)
      {
        // 寻找point_world的最近邻的平面点
        ikdtree_.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
        // 判断是否是有效匹配点，与loam系列类似，要求特征点最近邻的地图点数量>阈值，距离<阈值  满足条件的才置为true
        point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                                            : true;
      }
      if (!point_selected_surf[i])
        continue; // 如果该点不满足条件  不进行下面步骤

      Eigen::Matrix<float, 4, 1> pabcd; // 平面点信息
      point_selected_surf[i] = false;   // 将该点设置为无效点，用来判断是否满足条件
      // 拟合平面方程ax+by+cz+d=0并求解点到平面距离
      if (esti_plane(pabcd, points_near, 0.1f))
      {
        float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3); // 当前点到平面的距离
        float s = 1 - 0.9 * fabs(pd2) / sqrt(p_l.norm());                                                      // 如果残差大于经验阈值，则认为该点是有效点  简言之，距离原点越近的lidar点  要求点到平面的距离越苛刻

        if (s > 0.9) // 如果残差大于阈值，则认为该点是有效点
        {
          point_selected_surf[i] = true;
          normvec->points[i].x = pabcd(0); // 存储平面的单位法向量  以及当前点到平面距离
          normvec->points[i].y = pabcd(1);
          normvec->points[i].z = pabcd(2);
          normvec->points[i].intensity = pd2;
        }
      }
    }

    int effct_feat_num = 0; // 有效特征点的数量
    for (int i = 0; i < cloud_size; i++)
    {
      if (point_selected_surf[i]) // 对于满足要求的点
      {
        laserCloudOri->points[effct_feat_num] = cloud->points[i];   // 把这些点重新存到laserCloudOri中
        corr_normvect->points[effct_feat_num] = normvec->points[i]; // 存储这些点对应的法向量和到平面的距离
        effct_feat_num++;
      }
    }

    if (effct_feat_num < 1)
    {
      ekfom_data.valid = false;
      ROS_WARN("No Effective Points! \n");
      return;
    }

    // 雅可比矩阵H和残差向量的计算
    ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 24);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
      Vec3 point_(laserCloudOri->points[i].x, laserCloudOri->points[i].y, laserCloudOri->points[i].z);
      Mat3 point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_);
      Vec3 point_I_ = state.Ril * point_ + state.til;
      Mat3 point_I_crossmat;
      point_I_crossmat << SKEW_SYM_MATRX(point_I_);

      // 得到对应的平面的法向量
      const PointType &norm_p = corr_normvect->points[i];
      Vec3 norm_vec(norm_p.x, norm_p.y, norm_p.z);

      // 计算雅可比矩阵H
      Vec3 C(state.rot.transpose() * norm_vec);
      Vec3 A(point_I_crossmat * C);
      if (extrinsic_est_)
      {
        Vec3 B(point_crossmat * state.Ril.transpose() * C);
        ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
      }
      else
      {
        ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }

      // 残差：点面距离
      ekfom_data.h(i) = -norm_p.intensity;
    }
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

  void pubCloud(const ros::Publisher &pub_cloud, PointCloud::Ptr &cloud, double time)
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
  void setOdomMsg(T &out)
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

  void pubOdom(const ros::Publisher &pub, const ESEKF::esekf &kf, double time)
  {
    nav_msgs::Odometry odom;

    odom.header.frame_id = "map";
    odom.child_frame_id = "imu";
    odom.header.stamp = ros::Time().fromSec(time);
    setOdomMsg(odom.pose);
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
};
