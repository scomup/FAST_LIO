#pragma once

#include "so3_math.hpp"
#include <common_lib.h>

// 该hpp主要包含：广义加减法，前向传播主函数，计算特征点残差及其雅可比，ESKF主函数
namespace ESEKF
{
  const double epsi = 0.001; // ESKF迭代时，如果dx<epsi 认为收敛

  struct HData
  {
    bool valid;                                                // 有效特征点数量是否满足要求
    bool converge;                                             // 迭代时，是否已经收敛
    Eigen::Matrix<double, Eigen::Dynamic, 1> h;                // 残差	(公式(14)中的z)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_x; // 雅可比矩阵H (公式(14)中的H)
  };

  // the location of state
  constexpr int SZ = 24; // state size
  constexpr int NZ = 12; // noise size
  // the location of state
  constexpr int L_P = 0;
  constexpr int L_R = 3;
  constexpr int L_Rli = 6;
  constexpr int L_Tli = 9;
  constexpr int L_V = 12;
  constexpr int L_Bw = 15;
  constexpr int L_Ba = 18;
  constexpr int L_G = 21;
  // the location of noise
  constexpr int L_Nw = 0;
  constexpr int L_Na = 3;
  constexpr int L_Nbw = 6;
  constexpr int L_Nba = 9;

  // Input u (IMU)
  struct InputU
  {
    Eigen::Vector3d acc = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d gyro = Eigen::Vector3d(0, 0, 0);
  };

  struct State
  {
    Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);    // imu postion in world frame
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity(); // imu rotation in world frame
    Eigen::Matrix3d Ril = Eigen::Matrix3d::Identity(); // rotation from lidar to imu
    Eigen::Vector3d til = Eigen::Vector3d(0, 0, 0);    // translation from lidar to imu
    Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d bg = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d ba = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d grav = Eigen::Vector3d(0, 0, -G_m_s2);

    // plus for state
    State plus(Eigen::Matrix<double, SZ, 1> &f) const
    {
      State r;
      r.pos = this->pos + f.segment<3>(L_P);
      r.rot = this->rot * Eigen::Quaterniond(SO3Expmap(f.segment<3>(L_R)));
      r.Ril = this->Ril * Eigen::Quaterniond(SO3Expmap(f.segment<3>(L_Rli)));
      r.til = this->til + f.segment<3>(L_Tli);
      r.vel = this->vel + f.segment<3>(L_V);
      r.bg = this->bg + f.segment<3>(L_Bw);
      r.ba = this->ba + f.segment<3>(L_Ba);
      r.grav = this->grav + f.segment<3>(L_G);
      return r;
    }

    // minus for state
    Eigen::Matrix<double, SZ, 1> minus(const State &x2) const
    {
      Eigen::Matrix<double, SZ, 1> r;
      r.segment<3>(L_P) = this->pos - x2.pos;
      r.segment<3>(L_R) = SO3Logmap(x2.rot.transpose() * this->rot);
      r.segment<3>(L_Rli) = SO3Logmap(x2.Ril.transpose() * this->Ril);
      r.segment<3>(L_Tli) = this->til - x2.til;
      r.segment<3>(L_V) = this->vel - x2.vel;
      r.segment<3>(L_Bw) = this->bg - x2.bg;
      r.segment<3>(L_Ba) = this->ba - x2.ba;
      r.segment<3>(L_G) = this->grav - x2.grav;
      return r;
    }

    Eigen::Affine3d getAffine3d() const
    {
      Eigen::Affine3d affine = Eigen::Translation3d(pos) * rot;
      return affine;
    }
  };

}
