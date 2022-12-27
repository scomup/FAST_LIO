#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct PointXYZIT
{
  PCL_ADD_POINT4D; // quad-word XYZ
  float intensity; //< laser intensity reading
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, time, time));


