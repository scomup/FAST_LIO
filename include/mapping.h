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


#pragma once


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "state.h"
#include "esekf.h"
#include "ikd_Tree.h"

#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

class Mapping
{
public:
  Mapping(bool extrinsic_est, double filter_size_map);

  void setState(ESEKF::State &state);

  void pointL2W(PointType const *const pi, PointType *const po);

  template <typename T>
  void pointL2W(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);

  void updateMapArea(Vec3 &pos_LiD);

  bool initMap(const PointCloud::Ptr &cloud);

  void hModel(ESEKF::HData &ekfom_data, ESEKF::State &state, PointCloud::Ptr &cloud);

  void updateMap(PointCloud::Ptr cloud);

  void pubCloud(const ros::Publisher &pub_cloud, PointCloud::Ptr &cloud, double time);

  template <typename T>
  void setOdomMsg(T &out);

  void pubOdom(const ros::Publisher &pub, const ESEKF::Esekf &kf, double time);

private:
  double filter_size_map_ = 0;

  bool extrinsic_est_ = true;

  std::vector<PointVector> neighbor_array_;

  KD_TREE<PointType> ikdtree_;

  ESEKF::State state_;

  PointCloud::Ptr norm_vec;               // 特征点在地图中对应的平面参数(平面的单位法向量,以及当前点到平面距离)
  PointCloud::Ptr laser_cloud_origin;     // 有效特征点
  PointCloud::Ptr corr_norm_vect;         // 有效特征点对应点法相量
  bool point_selected_surf[100000] = {1}; // 判断是否是有效特征点
};
