#include "common_lib.h"
#include "so3_math.h"


float calc_dist(PointType p1, PointType p2)
{
  float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
  return d;
}

bool esti_plane(Eigen::Matrix<double, 4, 1> &pca_result, const PointVector &point, const double threshold)
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



